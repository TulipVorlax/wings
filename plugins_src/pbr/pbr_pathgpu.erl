%%
%%  pbr_renderer.erl
%%
%%     Pbr renderer handling
%%
%%  Copyright (c) 2010 Dan Gudmundsson
%%

-module(pbr_pathgpu).
-export([start/2]).

-record(ropt, 
	{max_path_depth,
	 rr_depth,
	 rr_imp_cap,
	 sampler,
	 filter}).
	
-record(sampler, {type, opts=[]}).
-record(filter,  {type, dim={0.0,0.0}, opts=[]}).

start(Attrs, RS) ->
    ROpt = #ropt{max_path_depth = proplists:get_value(max_path_depth, Attrs, 5),
		 rr_depth       = proplists:get_value(rr_depth, Attrs, 3),
		 rr_imp_cap     = proplists:get_value(rr_imp_cap, Attrs, 0.125),
		 sampler        = get_sampler(Attrs),
		 filter         = get_filter(Attrs)},
    init_render(ROpt, Rs),
    start_processes(ROpt, RS).

start_processes(Ropt, Rs) ->
    NoThreads = erlang:system_info(schedulers),
    random:seed(now()),
    render(1, random:uniform(1 bsl 32), 1.00, Ropt, Rs),
    ok.

render(Id, Seed, Start, Opt, State = #renderer{cl=CL}) ->
    Context = wings_cl:get_context(CL),
    RWBuff = fun(Sz) -> 
		     {ok, Buff} = cl:create_buffer(Context, [read_write], Sz),
		     Buff
	     end,
    StaticBuff = fun(Bin) ->
			 {ok, Buff} = cl:create_buffer(Context, 
						       [read_only, copy_host_ptr], 
						       binary_size(Bin), Bin),
			 Buff
		 end,
    {X,Y} = pbr_film:resolution(State),
    %% Work areas
    Rays = RWBuff(?RAYBUFFER_SZ),    
    Hits = RWBuff(?RAYHIT_SZ*?MAX_RAYS),
    FrameBuffer = RWBuff(?RAYHIT_SZ*X*Y),
    %% Static Scene buffers
    {Face2Mesh, Mesh2Mat, Mats} = pbr_scene:mesh2mat(State),
    MeshIds   = StaticBuff(Face2Mesh),  
    Mesh2Mat  = StaticBuff(Mesh2Mat),
    Materials = StaticBuff(pbr_mat:pack_materials(Mats)),
    Colors    = StaticBuff(pbr_scene:vertex_colors(State)),
    Normals   = StaticBuff(pbr_scene:normals(State)),
    Triangles = StaticBuff(pbr_scene:triangles(State)),
    Vertices  = StaticBuff(pbr_scene:vertices(State)),

    %% Static Lights
    %% AreaLight = StatBuff(pbr_light:bin_area(State)),
    %% InfiniteLight = ...
    
    %% Textures 

    

    ok.

init_render(Opts, Rs) ->
    ok


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
	     opts=[{rate   = proplists:get_value(metropolis_rate,Attrs,0.4),
		    reject = proplists:get_value(metropolis_reject,Attrs,512)
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


