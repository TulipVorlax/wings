%%
%%  pbr_renderer.erl
%%
%%     Pbr renderer handling
%%
%%  Copyright (c) 2010 Dan Gudmundsson
%%

-module(pbr_pathgpu).
-export([start/2]).

-include("pbr.hrl").

-record(ropt, 
	{max_path_depth,
	 rr_depth,
	 rr_imp_cap,
	 sampler,
	 filter, 
	 lens_r
	}).
	
-record(sampler, {type, opts=[]}).
-record(filter,  {type, dim={0.0,0.0}, opts=[]}).

start(Attrs, RS) ->
    ROpt = #ropt{max_path_depth = proplists:get_value(max_path_depth, Attrs, 5),
		 rr_depth       = proplists:get_value(rr_depth, Attrs, 3),
		 rr_imp_cap     = proplists:get_value(rr_imp_cap, Attrs, 0.125),
		 sampler        = get_sampler(Attrs),
		 filter         = get_filter(Attrs),
		 lens_r         = proplists:get_value(aperture, Attrs, 0.0)
		},
    start_processes(ROpt, RS).


start_processes(Ropt, Rs) ->
    %% NoThreads = erlang:system_info(schedulers),
    random:seed(now()),
    init_render(1, random:uniform(1 bsl 32), 0.00, Ropt, Rs),
    normal.

init_render(Id, Seed, Start, Opt, State = #renderer{cl=CL0}) ->
    Context = wings_cl:get_context(CL0),
    RWBuff = fun(Sz) -> 
		     {ok, Buff} = cl:create_buffer(Context, [read_write], Sz),
		     Buff
	     end,
    StaticBuff = fun(Bin) when is_binary(Bin) ->
			 {ok, Buff} = cl:create_buffer(Context, 
						       [read_only, copy_host_ptr], 
						       byte_size(Bin), Bin),
			 Buff;
		    (_) -> false
		 end,
    {X,Y} = pbr_film:resolution(State),
    %% Work areas
    RaysB = RWBuff(?RAYBUFFER_SZ),    
    HitsB = RWBuff(?RAYHIT_SZ*?MAX_RAYS),
    FrameBufferB = RWBuff(?RAYHIT_SZ*X*Y),
    %% Static Scene buffers
    {Face2Mesh, Mesh2Mat, Mats} = pbr_scene:mesh2mat(State),
    MeshIdsB   = StaticBuff(Face2Mesh),
    Mesh2MatB  = StaticBuff(Mesh2Mat),
    MaterialsB = StaticBuff(pbr_mat:pack_materials(Mats)),
    ColorsB    = StaticBuff(pbr_scene:vertex_colors(State)),
    NormalsB   = StaticBuff(pbr_scene:normals(State)),
    TrianglesB = StaticBuff(pbr_scene:triangles(State)),
    VerticesB  = StaticBuff(pbr_scene:vertices(State)),

    %% Lights
    %% AreaLight = StatBuff(pbr_light:pack_area(State)),
    AreaLightN = 0,  %% Fixme
    %% InfiniteLight = ...
    InfLight = false,
    Lights = pbr_scene:get_lights(State),
    SunLightB = StaticBuff(pbr_light:pack_light(sunlight, Lights)),
    SkyLightB = StaticBuff(pbr_light:pack_light(skylight, Lights)),
    
    SunLightB /= false orelse SkyLightB /= false orelse exit(no_light),
    
    %% Textures 
    TexMapRGBB = false,
    TexMapAlphaB = false,
    TexMapDescB = false,
    MeshTexsB = false,
    MeshBumpsB = false,
    MeshBumpsScaleB = false,
    UvsB = false,

    CamB = pbr_camera:pack_matrixes(State),
    
    io:format("Everything packed~n",[]),
    
    InDirL = AreaLightN > 0 orelse SunLightB /= false,
    TexA = TexMapAlphaB /= false,
    CamLens = Opt#ropt.lens_r,  %% (scene->camera->lensRadius > 0.f)
    TaskSize = calc_task_size(Opt, InDirL, TexA, CamLens),
    TaskBuff = RWBuff(TaskSize*?MAX_RAYS),
    {CameraHither, CameraYon} = pbr_camera:get_near_far(State),
    {_WCenter, WRad} = e3d_bv:sphere(pbr_scene:bb(State)),
    StartLine = trunc(Start*Y),
    Ps = [param("PARAM_STARTLINE", StartLine),
	  param("PARAM_TASK_COUNT", ?MAX_RAYS),
	  param("PARAM_IMAGE_WIDTH", X),
	  param("PARAM_IMAGE_HEIGHT", Y),
	  param("PARAM_RAY_EPSILON", ?RAY_EPS),
	  param("PARAM_CLIP_YON", CameraYon),
	  param("PARAM_CLIP_HITHER", CameraHither),
	  param("PARAM_SEED", Seed),
	  param("PARAM_MAX_PATH_DEPTH", Opt#ropt.max_path_depth),
	  param("PARAM_RR_DEPTH",  Opt#ropt.rr_depth),
	  param("PARAM_RR_CAP", Opt#ropt.rr_imp_cap),
	  param("PARAM_WORLD_RADIUS", WRad*1.01)
	 ],
    MatPs = [mat_param(Mat) || Mat <- pbr_mat:mat_types(Mats)],
    MatPs == [] andalso exit(no_materials),
    CamPs = if CamLens -> 
		    CamFocDist = pbr_camera:get_fdist(State),
		    [" -D PARAM_CAMERA_HAS_DOF",
		     param("PARAM_CAMERA_LENS_RADIUS", CamLens),
		     param("PARAM_CAMERA_FOCAL_DISTANCE", CamFocDist)];
	       true -> []
	    end,
    LightPs = light_params(SkyLightB, SunLightB, AreaLightN, InfLight),
    TexPs = [],
    
    FilterPs = filter_params(Opt#ropt.filter),
    %% PixelAtomics = " -D PARAM_USE_PIXEL_ATOMICS";
    SamplerPs = sampler_params(Opt#ropt.sampler),

    Params = lists:append([Ps, MatPs, CamPs, LightPs, TexPs, FilterPs, SamplerPs]),
    CL = compile(CL0, Params),
    %% FIXME check kernel workgroup sizes 
    

    CL.

compile(CL, Params) -> 
    Fs = ["pbr/pathgpu2_kernel_datatypes.cl", 
	  "pbr/pathgpu2_kernel_core.cl",
	  "pbr/pathgpu2_kernel_filters.cl",
	  "pbr/pathgpu2_kernel_scene.cl",
	  "pbr/pathgpu2_kernel_samplers.cl",
	  "pbr/pathgpu2_kernel_kernels.cl"],
    io:format("Defines: ~p~n",[Params]),
    wings_cl:compile(Fs, Params, CL).


-define(ifelse(A,B,C), if (A) -> (B); true -> (C) end).
-define(FSz, 4).
-define(ISz, 4).
-define(PathStateDLSz, undefined_sz).
-define(PathStateSz, undefined_sz).

param(Str, Value) when is_integer(Value) -> 
    io_lib:format(" -D ~s=~pf", [Str,Value]);
param(Str, Value) when is_float(Value) -> 
    io_lib:format(" -D ~s=~ff", [Str,Value]).

mat_param(matte)       -> " -D PARAM_ENABLE_MAT_MATTE";
mat_param(arealight)   -> " -D PARAM_ENABLE_MAT_AREALIGHT";
mat_param(mirror)      -> " -D PARAM_ENABLE_MAT_MIRROR";
mat_param(glass)       -> " -D PARAM_ENABLE_MAT_GLASS";
mat_param(mattemirror) -> " -D PARAM_ENABLE_MAT_MATTEMIRROR";
mat_param(metal)       -> " -D PARAM_ENABLE_MAT_METAL";
mat_param(mattemetal)  -> " -D PARAM_ENABLE_MAT_MATTEMETAL";
mat_param(alloy)       -> " -D PARAM_ENABLE_MAT_ALLOY";
mat_param(archglass)   -> " -D PARAM_ENABLE_MAT_ARCHGLASS".

light_params(SkyLightB, SunLightB, AreaLightN, InfLight) ->
    [?ifelse(InfLight /= false, " -D PARAM_HAS_INFINITELIGHT", []),
     ?ifelse(SkyLightB /= false, " -D PARAM_HAS_SKYLIGHT", []),
     if SunLightB /= false, AreaLightN == 0 ->
	     " -D PARAM_HAS_SUNLIGHT -D " ++
		 " -D PARAM_DL_LIGHT_COUNT=0";	     
	SunLightB ->
	     " -D PARAM_HAS_SUNLIGHT";
	AreaLightN ->
	     " -D PARAM_DIRECT_LIGHT_SAMPLING" ++
		 param("PARAM_DL_LIGHT_COUNT", AreaLightN)
     end].

filter_params(#filter{type=none}) ->
    " -D PARAM_IMAGE_FILTER_TYPE=0";
filter_params(#filter{type=box, dim={X,Y}}) ->
    " -D PARAM_IMAGE_FILTER_TYPE=1" ++
	param("PARAM_IMAGE_FILTER_WIDTH_X", X) ++
	param("PARAM_IMAGE_FILTER_WIDTH_Y", Y);
filter_params(#filter{type=gaussion, dim={X,Y}, opts=[{alpha,GA}]}) ->
    " -D PARAM_IMAGE_FILTER_TYPE=2" ++
	param("PARAM_IMAGE_FILTER_WIDTH_X", X) ++
	param("PARAM_IMAGE_FILTER_WIDTH_Y", Y) ++
	param("PARAM_IMAGE_FILTER_GAUSSIAN_ALPHA",GA);
filter_params(#filter{type=mitchell, dim={X,Y}, opts=[{b,B}, {c,C}]}) ->
    " -D PARAM_IMAGE_FILTER_TYPE=3" ++
	param("PARAM_IMAGE_FILTER_WIDTH_X", X) ++
	param("PARAM_IMAGE_FILTER_WIDTH_Y", Y) ++
	param("PARAM_IMAGE_FILTER_MITCHELL_B",B) ++
	param("PARAM_IMAGE_FILTER_MITCHELL_C",C).

sampler_params(#sampler{type=inlined_random}) ->
    " -D PARAM_SAMPLER_TYPE=0";    
sampler_params(#sampler{type=random}) ->
    " -D PARAM_SAMPLER_TYPE=1";
sampler_params(#sampler{type=metropolis, opts=[{rate, Rate}, {reject=Rej}]}) ->
    " -D PARAM_SAMPLER_TYPE=2"++
	param("PARAM_SAMPLER_METROPOLIS_LARGE_STEP_RATE", Rate) ++
	param("PARAM_SAMPLER_METROPOLIS_MAX_CONSECUTIVE_REJECT", Rej);
sampler_params(#sampler{type=stratified, opts=[{X,Y}]}) ->
        " -D PARAM_SAMPLER_TYPE=3"++
	param("PARAM_SAMPLER_STRATIFIED_X_SAMPLES", X) ++
	param("PARAM_SAMPLER_STRATIFIED_Y_SAMPLES", Y).

calc_task_size(#ropt{max_path_depth=PathDepth,sampler=Sampler},InDirL, TexA, CamL) ->
    Seed = 3*?ISz,    
    SamplerSz = sampler_size(Sampler, PathDepth, InDirL, TexA, CamL),
    PathStateSz = ?ifelse(InDirL, ?PathStateDLSz, ?PathStateSz),
    Seed + SamplerSz + PathStateSz.

sampler_size(#sampler{type=Type,opts=Opts}, PathDepth, InDirL, TexA, CamL) ->
    CameraSz = 0, %% Fixme
    DataEyePath = ?FSz * 2 + CameraSz, %% IDX_SCREEN_X, IDX_SCREEN_Y
    DataPerPath = 
	?ifelse(TexA, ?FSz, 0) +
	?FSz * 3 + %% IDX_BSDF_X, IDX_BSDF_Y, IDX_BSDF_Z
	%% IDX_DIRECTLIGHT_X, IDX_DIRECTLIGHT_Y, IDX_DIRECTLIGHT_Z
	?ifelse(InDirL, ?FSz*3, 0) + 
	%% IDX_RR
	?FSz,
    DataSz = 
	case Type of
	    inlined_random -> 
		?FSz*2;
	    metropolis -> 
		?FSz*2+?ISz*5+?FSz*3+2*(DataEyePath+DataPerPath*PathDepth);
	    _ -> 
		DataEyePath+DataPerPath*PathDepth
	end,
    case Type of
	metropolis -> 
	    DataSz + ?FSz*3;
	stratified -> 
	    ?ISz + DataSz + ?FSz*3 + 
		stratified_sampler_size(Opts, InDirL, TexA, CamL);
	_ -> 
	    ?ISz + DataSz + ?FSz*3
    end.

stratified_sampler_size({X,Y}, InDirL, TexA, CamL) ->
    %% stratifiedScreen2D
    ?FSz * X * Y * 2 +
	%% stratifiedDof2D
	?ifelse(CamL, ?FSz * X * Y * 2, 0) +
	%% stratifiedAlpha1D
	?ifelse(TexA, ?FSz * X, 0) +
	%% stratifiedBSDF2D
	?FSz * X * Y * 2 +
	%% stratifiedBSDF1D
	?FSz * X +
	%% stratifiedLight2D
	%% stratifiedLight1D
	?ifelse(InDirL, ?FSz *X*Y*2 + ?FSz *X, 0).

	



%% Helpers

get_sampler(Attrs) ->
    get_sampler(proplists:get_value(sampler, Attrs, metropolis), Attrs).

get_sampler(random, _Attrs) ->
    #sampler{type=random};
get_sampler(stratified, Attrs) ->
    Samples = proplists:get_value(stratified_samples, Attrs, {3,3}),
    #sampler{type=stratified, opts=[Samples]};
get_sampler(metropolis, Attrs) ->
    #sampler{type=metropolis,
	     opts=[{rate,proplists:get_value(metropolis_rate,Attrs,0.4),
		    reject,proplists:get_value(metropolis_reject,Attrs,512)
		   }]}.

get_filter(Attrs) ->
    get_filter(proplists:get_value(filter, Attrs, none), Attrs).

get_filter(none, _Attrs) ->
    none;
get_filter(box, Attrs) ->
    Dim = proplists:get_value(filter_dim, Attrs, {1.5,1.5}),
    #filter{type=box, dim=Dim};
get_filter(gaussian, Attrs) ->
    Dim = proplists:get_value(filter_dim, Attrs, {1.5,1.5}),
    Alpha = proplists:get_value(filter_alpha, Attrs, 2.0),
    #filter{type=gaussian, dim=Dim, opts=[{alpha,Alpha}]};
get_filter(mitchell, Attrs) ->
    Dim = proplists:get_value(filter_dim, Attrs, {1.5,1.5}),
    B = proplists:get_value(filter_b, Attrs, 1/3),
    C = proplists:get_value(filter_c, Attrs, 1/3),
    #filter{type=mitchell, dim=Dim, opts=[{b,B},{c,C}]}.


