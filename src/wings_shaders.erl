%%
%%  wings_shaders.erl --
%%
%%     Support for vertex & fragment shaders (for cards with OpenGL 2.0).
%%
%%  Copyright (c) 2001-2011 Bjorn Gustavsson
%%
%%  See the file "license.terms" for information on usage and redistribution
%%  of this file, and for a DISCLAIMER OF ALL WARRANTIES.
%%
%%     $Id$
%%

-module(wings_shaders).
-export([init/0, read_texture/1, set_active/1]).

-define(NEED_OPENGL, 1).
-include("wings.hrl").
-include("e3d_image.hrl").

init() ->
    wings_pref:set_default(hl_lightpos, {3000.0, 10000.0, 1000.0}),
    wings_pref:set_default(hl_skycol, {0.95,0.95,0.90}),
    wings_pref:set_default(hl_groundcol, {0.026,0.024,0.021}),
    HL = [{"LightPosition", wings_pref:get_value(hl_lightpos)},
	  {"SkyColor", wings_pref:get_value(hl_skycol)},
	  {"GroundColor", wings_pref:get_value(hl_groundcol)}],
    Programs = {{make_prog("hemilight", HL), "Hemispherical Lighting"},
		{make_prog("gooch"), "Gooch Tone"},
		{make_prog("toon"), "Toon"},
		{make_prog("brick"), "Brick"},
		{make_prog("envmap"), "Environment Mapping"},
		{make_prog("vertex_color", [{"Flag", 0}]), "Vertex Normals Color"},
		{make_prog("vertex_color", [{"Flag", 1}]), "Face Normals Color"},
		{make_prog("spherical_ao"), "Spherical Ambient Occlusion"},
		{make_prog("depth"), "Depth"},
		{make_prog("harmonics", [{"Type", 5}]), "Spherical Harmonics 5"},
		{make_prog("harmonics", [{"Type", 8}]), "Spherical Harmonics 8"},
		{make_prog("harmonics", [{"Type", 9}]), "Spherical Harmonics 9"}},
    ?CHECK_ERROR(),
    gl:useProgram(0),
    put(light_shaders, Programs),
    case wings_pref:get_value(active_shader) > tuple_size(Programs) of
	true -> set_active(1);
	false -> ok
    end,
    io:format("Using GPU shaders.\n").

set_active(Id) ->
    wings_pref:set_value(active_shader, Id),
    ok.

read_texture(FileName) ->
    Path = filename:join(wings_util:lib_dir(wings), "textures"),
    NewFileName = filename:join(Path, FileName),
    ImgRec = e3d_image:load(NewFileName, [{order,lower_left}]),
    ImgRec.

read_shader(FileName) ->
    Path = filename:join(wings_util:lib_dir(wings), "shaders"),
    NewFileName = filename:join(Path, FileName),
    {ok,Bin} = file:read_file(NewFileName),
    Bin.

make_prog(Name) ->
    make_prog(Name, []).
make_prog(Name, Vars) ->
    Shv = wings_gl:compile(vertex, read_shader(Name ++ ".vs")),
    Shf = wings_gl:compile(fragment, read_shader(Name ++ ".fs")),
    Prog = wings_gl:link_prog([Shv,Shf],[{?TANGENT_ATTR, "wings_tangent"}]),
    gl:useProgram(Prog),
    envmap(Name, Prog),
    wings_gl:set_uloc(Prog, "DiffuseMap", ?DIFFUSE_MAP_UNIT),
    wings_gl:set_uloc(Prog, "NormalMap",  ?NORMAL_MAP_UNIT),
    [wings_gl:set_uloc(Prog, Var, Val) || {Var,Val} <- Vars],
    Prog.

envmap("envmap", Prog) -> 
    FileName = "grandcanyon.png",
    EnvImgRec = read_texture(FileName),
    #e3d_image{width=ImgW,height=ImgH,image=ImgData} = EnvImgRec,
    [TxId] = gl:genTextures(1),
    gl:activeTexture(?GL_TEXTURE0 + ?ENV_MAP_UNIT),
    gl:bindTexture(?GL_TEXTURE_2D, TxId),
    gl:texParameteri(?GL_TEXTURE_2D, ?GL_TEXTURE_WRAP_S, ?GL_REPEAT),
    gl:texParameteri(?GL_TEXTURE_2D, ?GL_TEXTURE_WRAP_T, ?GL_REPEAT),
    gl:texParameteri(?GL_TEXTURE_2D, ?GL_TEXTURE_MAG_FILTER, ?GL_LINEAR),
    gl:texParameteri(?GL_TEXTURE_2D, ?GL_TEXTURE_MIN_FILTER, ?GL_LINEAR),
    gl:texImage2D(?GL_TEXTURE_2D, 0, ?GL_RGB, ImgW, ImgH, 0, ?GL_RGB,
		  ?GL_UNSIGNED_BYTE, ImgData),
    wings_gl:set_uloc(Prog, "EnvMap", ?ENV_MAP_UNIT),
    gl:activeTexture(?GL_TEXTURE0);
envmap(_, _) -> ok.
