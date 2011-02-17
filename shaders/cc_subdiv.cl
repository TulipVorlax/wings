// -*- c++ -*-
// @Author: <dgud@users.sf.net>
// @copyright (C) 2010
// @doc Catmull Clark subdivision

#define PL_UNITS 4

typedef struct {
  int start;
  int len;
} FaceIndex;

typedef struct {
  int start;
  int len;
  int vab;
} VabIndex;

typedef struct {
    float x;
    float y;
    float z;
} float3;

typedef struct {
    float x;
    float y;
    float z;
    float u;
    float v;
} color_uv;

// Predefs
bool find_vertices(int v1, int v2, int4 face, int *v1pos, int *v2pos);
void find_faces(int V0, int V1, FaceIndex Fi, __global int *Fs, 
		int * F1, int *F2, int *CCW);

float4 sum_smooth(uint id, __global float4 *Smooth, const int noVsOut);

void get_normal(int n, int face, int vertex, 
		__global float4 *VsIn,
		__global float4 *Out, __global float4 *Out2, 
		__global float4 *Smooth, __global float3 *Vab, int noVsOut);

inline float3* float4_to_float3(float4 vec, float3* col) {
    col->x = vec.x;
    col->y = vec.y;
    col->z = vec.z;
    return col;
}

inline float4 float3_to_float4(float3 col) {
    float4 vec = {col.x, col.y, col.z, 0.0};
    return vec;
}

inline void add_color_uv(color_uv in, color_uv *out) {
    out->x += in.x;
    out->y += in.y;
    out->z += in.z;
    out->u += in.u;
    out->v += in.v;
}

inline void div_color_uv(float in, color_uv *out) {
    out->x /= in;
    out->y /= in;
    out->z /= in;
    out->u /= in;
    out->v /= in;
}

inline void aver_color_uv(color_uv in1, color_uv in2, color_uv *aver) {
    aver->x = (in1.x + in2.x) / 2.0;
    aver->y = (in1.y + in2.y) / 2.0;
    aver->z = (in1.z + in2.z) / 2.0;
    aver->u = (in1.u + in2.u) / 2.0;
    aver->v = (in1.v + in2.v) / 2.0;
}

__kernel void gen_faces(
			__global float4 *VsIn,
			__global int *FsIn,
			__global FaceIndex *FiIn,
			__global float4 *VsOut,
			__global int4 *FsOut,
			//__global int *locks,
			const uint noFs,
			const uint noVs
			)
{
  int i;
  const int face_id = get_global_id(0);
  if (face_id >= noFs)
      return;
  const FaceIndex fi = FiIn[face_id];
  float4 center = {0.0,0.0,0.0,0.0};
  
  for(i=0; i < fi.len; i++) {
      center.xyz += VsIn[FsIn[fi.start+i]].xyz;
  }

  center /= i;
  // Create new center vertex
  const uint ov_id = noVs + face_id;
  center.w = fi.len*4.0;  // Valance = faceVs and hard_edge count = 0 (Valance << 2)
  VsOut[ov_id] = center;

  center.w = 0.0;

  for(i=0; i < fi.len; i++) {
    int id = fi.start+i;
    int v_id = FsIn[id];
    // Add center to all face verts    
    // Create Faces    
    FsOut[id].x = v_id;
    FsOut[id].y = -5; 
    FsOut[id].z = ov_id; 
    FsOut[id].w = -5;
  }
}

__kernel void add_center(
			 __global int *FsIn,
			 __global FaceIndex *FiIn,
			 __global float4 *VsOut,
			 const uint noFs,
			 const uint noVsIn,
			 const uint noVsOut
			 )
{
  int i, face_id, stop;
  const int id = get_global_id(0);
  if (id >= PL_UNITS) return;  // Should only run by PL_UNITS "threads"

  FaceIndex fi;
  uint v_id, ov_id;
  float4 center;
  float4 zero = {0.0,0.0,0.0,0.0}; 
  uint sect = noVsOut * id;
  uint chunk_sz = ceil((float) noFs / (float) PL_UNITS);
  face_id = id*chunk_sz;
  stop = min(face_id+chunk_sz, noFs);
  
  for(; face_id < stop; face_id++) {
      FaceIndex fi = FiIn[face_id];
      ov_id = noVsIn + face_id;
      center = VsOut[ov_id];
      center.w = 0.0;
      for(i=0; i < fi.len; i++) {
	  int v_id = FsIn[fi.start+i];
	  float4 v = VsOut[v_id];
	  uint he_c = trunc(v.w);
	  he_c = he_c % 4;
	  if(he_c < 2) {
	      VsOut[sect+v_id] += center;
	  } else if(he_c == 2) {
	      zero.w = v.w;
	      VsOut[v_id] = zero;
	  };
      }
  }
}

__kernel void gen_edges(__global float4 *VsIn,
			__global int *FsIn,
			__global int4 *EsIn,
			__global FaceIndex *FiIn,
			__global float4 *VsOut,
			__global int  *FsOut,
			__global int4 *EsOut,
			const uint noFs,
			const uint noVs,
			const uint noEs)
{
  int i;
  const int edge_id = get_global_id(0);
  if (edge_id >= noEs)
      return;
  float4 center = {0.0,0.0,0.0,0.0};
  int4 edge = EsIn[edge_id];
  int hard = 0;
  int ov_id = noVs+noFs+edge_id;
  int hov_id = ov_id;
  const int4 hole_edge = {-1,-1,-1,-1};

  if(edge.y < 0) { // Indicates edge in hole
      const int oe_id = edge_id*4;
      EsOut[oe_id+0] = hole_edge;
      EsOut[oe_id+1] = hole_edge;
      EsOut[oe_id+2] = hole_edge;
      EsOut[oe_id+3] = hole_edge;
      return;
  }

  if(edge.x < 0) {  // Indicates hard edge
      hard = 1;
      edge.x = -1-edge.x;
      hov_id = -1-ov_id;
  }

  center = VsIn[edge.x];  // V0
  center += VsIn[edge.y];  // V1
  if(hard) {
      center /= 2.0;
      center.w = 18.0; // Valance 4 and 2 hard edges ((4 << 2) | 2)
  } else {
      center += VsOut[noVs+edge.z]; // F1 Center
      center += VsOut[noVs+edge.w]; // F2 Center
      center /= 4.0;
      center.w = 16.0; // Valance 4 and 0 hard edges ((4 << 2) | 0)
  }
  
  // New vertex at edge center position
  VsOut[ov_id] = center;
  // Complete faces 
  int F11=-1,F12=-1,F21=-1,F22=-1, CCW1,CCW2;
  const int oe_id = edge_id*4;
  // Be sure to create faces with the correct order   
  if(edge.z >= 0) { // Edge is in a hole
      FaceIndex IF1 = FiIn[edge.z];
      find_faces(edge.x,edge.y,IF1,FsIn,&F11,&F12,&CCW1);
      const int4 e0 = {ov_id,noVs+edge.z,F11,F12};
      EsOut[oe_id+0] = e0;
      if(CCW1) {
	  FsOut[F11*4+1] = ov_id;
	  FsOut[F12*4+3] = ov_id;
      } else {
	  FsOut[F11*4+3] = ov_id;
	  FsOut[F12*4+1] = ov_id;
      }
  } else {
      EsOut[oe_id+0] = hole_edge;
  }
  if(edge.w >= 0) { // Edge is in a hole
      FaceIndex IF2 = FiIn[edge.w];
      find_faces(edge.x,edge.y,IF2,FsIn,&F21,&F22,&CCW2);
      const int4 e1 = {ov_id,noVs+edge.w,F21,F22};
      EsOut[oe_id+1] = e1;
      if(CCW2) {
	  FsOut[F21*4+1] = ov_id;
	  FsOut[F22*4+3] = ov_id;
      } else {
	  FsOut[F21*4+3] = ov_id;
	  FsOut[F22*4+1] = ov_id;
      }
  } else {
      EsOut[oe_id+1] = hole_edge;
  }
  // Hmm init only when declaring var on nvidia? 
  const int4 e2 = {hov_id,edge.x,F11,F21};
  EsOut[oe_id+2] = e2;
  const int4 e3 = {hov_id,edge.y,F12,F22};
  EsOut[oe_id+3] = e3;
}


__kernel void add_edge_verts(
			     __global float4 *VsIn,
			     __global float4 *VsOut,
			     __global int4 *EsIn,
			     const uint noEs,
			     const uint noVsOut
			     )
{
  const int thread = get_global_id(0);
  if (thread >= PL_UNITS) return;  // Should only run by one "thread"
  
  int id, stop;
  int4 edge;
  float4 v0,v1;
  int hard_v0=0, hard_v1=0;
  uint sect = noVsOut * thread;
  uint chunk_sz = ceil((float) noEs / (float) PL_UNITS);
  id = thread*chunk_sz;
  stop = min(id+chunk_sz, noEs);

  for(; id < stop; id++) {
      edge = EsIn[id];
      if(edge.y >= 0) {
	  if(edge.x < 0) { // Hard edge
	      edge.x = -1-edge.x;
	      v0 = VsIn[edge.x];
	      v0.w = 0.0;
	      VsOut[sect + edge.y] += v0;
	      v1 = VsIn[edge.y];
	      v1.w = 0.0;
	      VsOut[sect + edge.x] += v1;
	  } else { // Only add soft edges if vertex have <2 hardedges
	      v0 = VsIn[edge.x];
	      v1 = VsIn[edge.y];

	      hard_v0 = trunc(v0.w);	  
	      hard_v1 = trunc(v1.w);
	      hard_v0 = hard_v0 % 4;
	      hard_v1 = hard_v1 % 4;
	      if(hard_v1 < 2) {
		  v0.w = 0.0;
		  VsOut[sect + edge.y] += v0;
	      }
	      if(hard_v0 < 2) {
		  v1.w = 0.0;	  
		  VsOut[sect + edge.x] += v1;
	      }
	  }
      }    
  }
}

__kernel void move_verts(
			 __global float4 *VsIn,
			 __global float4 *VsOut,
			 const uint noInVs,
			 const uint noOutVs
			 )
{
  const int v_id = get_global_id(0);
  float4 v_out, zero = {0.0f,0.0f,0.0f,0.0f};
  if(v_id >= noOutVs)
    return;

  if(v_id >= noInVs) {
      // Copy buffer VsIn and VsOut should be equal
      // after this pass
      VsIn[v_id] = VsOut[v_id];
      return;
  }

  // Sum all sections and reset them afterwards
  v_out = VsOut[v_id];
  for(int i=1; i<PL_UNITS; i++) {
      uint id = noOutVs*i+v_id;
      v_out.xyz += VsOut[id];
      VsOut[id] = zero;
  }

  float4 v_in = VsIn[v_id];
  uint hc = trunc(v_in.w);
  uint vc = hc;
  hc = hc % 4;
  vc = vc / 4;
  if(hc < 2) {
    float a = 1.0/(vc*vc);
    float b = (vc-2.0)/vc;
    //  We started with Inpos remove it
    v_out -= v_in;
    v_out *= a;
    v_out += (v_in * b);
    v_out.w = v_in.w;
    VsOut[v_id] = v_out;
    VsIn[v_id] = v_out;
  } else if(hc == 2) {
    v_out += v_in * 6.0;
    v_out *= 1.0/8.0;
    v_out.w = v_in.w;
    VsOut[v_id] = v_out;
    VsIn[v_id] = v_out;
  } else {
    VsOut[v_id] = v_in;
  }
}

__kernel void subd_vcolor(
			 __global float3 *AsIn,
			 __global FaceIndex *FiIn,
			 __global float3 *AsOut,
			 const uint noFs
			 )
{
    int i;
    const int face_id = get_global_id(0);
    if (face_id >= noFs)
	return;
    const FaceIndex fi = FiIn[face_id];
    float4 aver, prev, curr, next, center = {0.0,0.0,0.0,0.0};
    float3 col;
    
    for(i=0; i < fi.len; i++) {
    	col = AsIn[fi.start+i];
    	center += float3_to_float4(col);
    }

    center /= i;

    prev = float3_to_float4(col);    
    curr = float3_to_float4(AsIn[fi.start]);

    for(i=0; i < fi.len; i++) {
    	int id = (fi.start+i);
	next = float3_to_float4(AsIn[fi.start+((i+1)%fi.len)]);
	
    	// Create face colors
    	id *= 4;
    	float4_to_float3(curr, &col);
    	AsOut[id+0] = col;
    	aver = (curr + next)/2.0;
    	float4_to_float3(aver, &col);
    	AsOut[id+1] = col;
    	float4_to_float3(center, &col);
    	AsOut[id+2] = col;
    	aver = (curr + prev)/2.0;
    	float4_to_float3(aver, &col);
    	AsOut[id+3] = col;
    	prev = curr;
    	curr = next;
    }
}

__kernel void subd_uv(
		      __global float2 *AsIn,
		      __global FaceIndex *FiIn,
		      __global float2 *AsOut,
		      const uint noFs
		      )
{
    int i;
    const int face_id = get_global_id(0);
    if (face_id >= noFs)
	return;
    const FaceIndex fi = FiIn[face_id];
    float2 aver, prev, curr, next, center = {0.0,0.0};
    
    for(i=0; i < fi.len; i++) {
    	curr = AsIn[fi.start+i];
    	center += curr;
    }

    center /= i;

    prev = curr;
    curr = AsIn[fi.start];

    for(i=0; i < fi.len; i++) {
    	int id = (fi.start+i);
	next = AsIn[fi.start+((i+1)%fi.len)];
	
    	// Create face uv's
    	id *= 4;
    	AsOut[id+0] = curr;
    	aver = (curr + next)/2.0;
    	AsOut[id+1] = aver;
    	AsOut[id+2] = center;
    	aver = (curr + prev)/2.0;
    	AsOut[id+3] = aver;
    	prev = curr;
    	curr = next;
    }
}

__kernel void subd_col_uv(
			 __global color_uv *AsIn,
			 __global FaceIndex *FiIn,
			 __global color_uv *AsOut,
			 const uint noFs
			 )
{
    int i;
    const int face_id = get_global_id(0);
    if (face_id >= noFs)
	return;
    const FaceIndex fi = FiIn[face_id];
    color_uv aver, prev, curr, next, center = {0.0,0.0,0.0,0.0,0.0};
    color_uv col;
    
    for(i=0; i < fi.len; i++) {
    	col = AsIn[fi.start+i];
    	add_color_uv(col, &center);
    }

    div_color_uv(i, &center);

    prev = col;
    curr = AsIn[fi.start];

    for(i=0; i < fi.len; i++) {
    	int id = (fi.start+i);
	next = AsIn[fi.start+((i+1)%fi.len)];
	
    	// Create face colors
    	id *= 4;    	
    	AsOut[id+0] = curr;
    	aver_color_uv(curr,next,&aver);
    	AsOut[id+1] = aver;
    	AsOut[id+2] = center;
	aver_color_uv(curr,prev,&aver);
    	AsOut[id+3] = aver;
    	prev = curr;
    	curr = next;
    }
}

__kernel void create_vab_all(
				__global float4 *Vs,
				__global int4 *Fs,
				__global float *Vab,
				const uint noFs
				)
{
    const int id = get_global_id(0);
    if(id >= noFs) 
	return;
    const int f_sz = 4*6;
    int4 face = Fs[id];
    float4 v1, v2, v3, v4, normal;
    v1 = Vs[face.x];
    v2 = Vs[face.y];
    v3 = Vs[face.z];
    v4 = Vs[face.w];
    normal = normalize(cross(v3-v1,v4-v2));
    // Output V1    
    Vab[id*f_sz+0] = v1.x;  Vab[id*f_sz+3] = normal.x;
    Vab[id*f_sz+1] = v1.y;  Vab[id*f_sz+4] = normal.y;
    Vab[id*f_sz+2] = v1.z;  Vab[id*f_sz+5] = normal.z;   
    // Output V2        
    Vab[id*f_sz+6] = v2.x;  Vab[id*f_sz+9]  = normal.x;
    Vab[id*f_sz+7] = v2.y;  Vab[id*f_sz+10] = normal.y;
    Vab[id*f_sz+8] = v2.z;  Vab[id*f_sz+11] = normal.z;
    // Output V3
    Vab[id*f_sz+12] = v3.x; Vab[id*f_sz+15] = normal.x;
    Vab[id*f_sz+13] = v3.y; Vab[id*f_sz+16] = normal.y;
    Vab[id*f_sz+14] = v3.z; Vab[id*f_sz+17] = normal.z;
    // Output V4
    Vab[id*f_sz+18] = v4.x; Vab[id*f_sz+21] = normal.x;
    Vab[id*f_sz+19] = v4.y; Vab[id*f_sz+22] = normal.y;
    Vab[id*f_sz+20] = v4.z; Vab[id*f_sz+23] = normal.z;    
}


__kernel void create_vab_sel(
			 __global float4 *VsIn,
			 __global int4 *FsIn,
			 __global VabIndex *FiIn,
			 __global float *Vab,
			 const int noFs
			 )
{
    const int id = get_global_id(0);
    if(id >= noFs)
	return;
    VabIndex fi = FiIn[id];
    int4 face;
    float4 v1, v2, v3, v4, normal;
    int vab, out = fi.vab*24;
    for(int i=0; i < fi.len; i++) {
	face = FsIn[fi.start+i];
	vab = out+i*24;
	v1 = VsIn[face.x];
	v2 = VsIn[face.y];
	v3 = VsIn[face.z];
	v4 = VsIn[face.w];
	normal = normalize(cross(v3-v1,v4-v2));
	// Output V1
	Vab[vab+0] = v1.x;  Vab[vab+3] = normal.x;
	Vab[vab+1] = v1.y;  Vab[vab+4] = normal.y;
	Vab[vab+2] = v1.z;  Vab[vab+5] = normal.z;   
	// Output V2        
	Vab[vab+6] = v2.x;  Vab[vab+9]  = normal.x;
	Vab[vab+7] = v2.y;  Vab[vab+10] = normal.y;
	Vab[vab+8] = v2.z;  Vab[vab+11] = normal.z;
	// Output V3
	Vab[vab+12] = v3.x; Vab[vab+15] = normal.x;
	Vab[vab+13] = v3.y; Vab[vab+16] = normal.y;
	Vab[vab+14] = v3.z; Vab[vab+17] = normal.z;
	// Output V4
	Vab[vab+18] = v4.x; Vab[vab+21] = normal.x;
	Vab[vab+19] = v4.y; Vab[vab+22] = normal.y;
	Vab[vab+20] = v4.z; Vab[vab+23] = normal.z;    
    }
}

__kernel void get_sel_vcolor(
			    __global VabIndex *FiIn,
			    __global float3 *AsIn,
			    __global float3 *AsOut,
			    const int noFs
			    )
{
    const int id = get_global_id(0);
    if(id >= noFs)
	return;
    VabIndex fi = FiIn[id];
    int in, out;
    for(int i=0; i < fi.len; i++) {
	in  = (fi.start+i)*4;
	out = (fi.vab  +i)*4;
	AsOut[out+0] = AsIn[in+0];
	AsOut[out+1] = AsIn[in+1];
	AsOut[out+2] = AsIn[in+2];
	AsOut[out+3] = AsIn[in+3];
    }
}

__kernel void get_sel_uv(
			 __global VabIndex *FiIn,
			 __global float8 *AsIn,
			 __global float8 *AsOut,
			 const int noFs
			 )
{
    const int id = get_global_id(0);
    if(id >= noFs)
	return;
    VabIndex fi = FiIn[id];
    int in, out;
    for(int i=0; i < fi.len; i++) {
	in  = (fi.start+i);
	out = (fi.vab  +i);
	AsOut[out] = AsIn[in];
    }
}

__kernel void get_sel_col_uv(
			    __global VabIndex *FiIn,
			    __global color_uv *AsIn,
			    __global color_uv *AsOut,
			    const int noFs
			    )
{
    const int id = get_global_id(0);
    if(id >= noFs)
	return;
    VabIndex fi = FiIn[id];
    int in, out;
    for(int i=0; i < fi.len; i++) {
	in  = (fi.start+i)*4;
	out = (fi.vab  +i)*4;
	AsOut[out+0] = AsIn[in+0];
	AsOut[out+1] = AsIn[in+1];
	AsOut[out+2] = AsIn[in+2];
	AsOut[out+3] = AsIn[in+3];
    }
}

__kernel void clearf(__global float *mem,
		     const int isz,
		     const int sz)
{
    const int id = get_global_id(0);
    if(id >= sz) return;
    int pos = id*isz;
    for(int i=0; i < isz; i++) {
	mem[pos+i] = 0.0f;
    }
}

__kernel void smooth_ns_pass0(
			      __global int4 *FsIn,
			      __global float3 *Vab,
			      __global float4 *Smooth,
			      const uint noFs,
			      const uint noVsOut
			      )
{
    const int thread = get_global_id(0);
    if(thread >= PL_UNITS) return;

    float4 normal;
    global float3 *temp;
    int4 face_vs;
    uint id, stop, sect = noVsOut * thread;
    uint chunk_sz = ceil((float) noFs / (float) PL_UNITS);
    id = thread*chunk_sz;
    stop = min(id+chunk_sz, noFs);
    
    for(; id < stop; id++) {
	temp = &Vab[(id*4*2)+1];
	normal.x = temp->x;
	normal.y = temp->y;
	normal.z = temp->z;
	normal.w = 0.0;
     
	face_vs = FsIn[id];
	Smooth[sect+face_vs.x] += normal;
	Smooth[sect+face_vs.y] += normal;
	Smooth[sect+face_vs.z] += normal;
	Smooth[sect+face_vs.w] += normal;
    }
}

__kernel void smooth_ns_pass1(__global int4 *EsIn,
			      __global int4 *FsIn,
			      __global float3 *Vab,
			      __global float4 *Out1,
			      __global float4 *Out2,
			      const int noEs)
{
    const int id = get_global_id(0);
    if(id >= noEs)
	return;
    int4 edge = EsIn[id];
    float4 normal = {0.0f,0.0f,0.0f,0.0f};
    __global float4 *Out;
    int v1,v2;

    if(edge.x >= 0) { // Soft edges
	if(find_vertices(edge.x, edge.y, FsIn[edge.z], &v1, &v2)) Out = Out1;
	else Out = Out2;
	normal = float3_to_float4(Vab[edge.w*4*2+1]);
	normal.w = 1.0f;
	Out[edge.z*4+v1] += normal;
	Out[edge.z*4+v2] += normal;

	if(find_vertices(edge.x, edge.y, FsIn[edge.w], &v1, &v2))  Out = Out1;
	else Out = Out2;
	normal = float3_to_float4(Vab[edge.z*4*2+1]);
	normal.w = 1.0f;
	Out[edge.w*4+v1] += normal;
	Out[edge.w*4+v2] += normal;
    } 
}

__kernel void smooth_ns(
			__global int4 *FsIn,
			__global float4 *VsIn,
			__global float4 *Smooth,
			__global float3 *Vab,
			__global float4 *Out1,
			__global float4 *Out2,
			const int noFs,
			const int noVsOut
			)
{
    const int id = get_global_id(0);
    if(id >= noFs)
	return;
    int4 face = FsIn[id];
    float4 normal;
    int vstart = id*4;
    
    get_normal(vstart+0, id, face.x, VsIn, Out1, Out2, Smooth, Vab, noVsOut);
    get_normal(vstart+1, id, face.y, VsIn, Out1, Out2, Smooth, Vab, noVsOut);
    get_normal(vstart+2, id, face.z, VsIn, Out1, Out2, Smooth, Vab, noVsOut);
    get_normal(vstart+3, id, face.w, VsIn, Out1, Out2, Smooth, Vab, noVsOut);
}

// Helpers

void get_normal(int n, int face, int vertex, 
		__global float4 *VsIn,
		__global float4 *Out, __global float4 *Out2, 
		__global float4 *Smooth, __global float3 *Vab, 
		int noVsOut)
{
    float4 pos = VsIn[vertex];
    int type = trunc(pos.w);
    type = type % 4;
    float4 normal;
    if(type == 0) {
	normal = sum_smooth(vertex,Smooth,noVsOut);
	Out[n] = normalize(normal);
	return;	
    }
    normal = Out[n];
    normal += Out2[n];
    normal.w = 0.0f;
    normal = float3_to_float4(Vab[(face*4*2)+1]) + normal;
    Out[n] = normalize(normal);
}

float4 sum_smooth(uint id, __global float4 *Smooth, const int noVsOut)
{
    float4 normal = {0.0f,0.0f,0.0f,0.0f};
    for(int i=0; i<PL_UNITS; i++) {
	normal += Smooth[i*noVsOut+id];
    }
    return normal;
}

// Find the order of faces so that vertices for a face
// comes in the ccw order
void find_faces(int V0, int V1, FaceIndex Fi, __global int *Fs, 
		int * F1, int *F2, int *CCW)
{
    int fva,fvb;
    fva = Fs[Fi.start];
    for(int i=Fi.start; i < (Fi.start+Fi.len); i++) {
	fvb = Fs[i+1];
	if(V0==fva) {
	    *F1 = i;
	    if(V1==fvb) {
		*F2 = i+1;
		*CCW = 1;
	    } else {
		*F2 = i+Fi.len-1;
		*CCW = 0;
	    }
	    return;
	}
	if(V1==fva) {
	    *F2 = i;
	    if(V0==fvb) {
		*F1 = i+1;
		*CCW = 0;
	    } else {
		*F1 = i+Fi.len-1;
		*CCW = 1;
	    }
	    return;
	}
	fva = fvb;
    };
    *F1 = -1;
    *F2 = -2;
    *CCW = 1;
}

bool find_vertices(int v1, int v2, int4 face, int *v1pos, int *v2pos)
{
    if(v1 == face.x) {
	*v1pos = 0;
	if(v2 == face.y) {*v2pos = 1; return true;}
	*v2pos = 3; return false;
    };
    if(v1 == face.y) {
	*v1pos = 1;
	if(v2 == face.z) {*v2pos = 2; return true;}
	*v2pos = 0; return false;
    };
    if(v1 == face.z) {
	*v1pos = 2;
	if(v2 == face.w) {*v2pos = 3; return true;}
	*v2pos = 1; return false;
    };
    *v1pos = 3;
    if(v2 == face.x) {*v2pos = 0; return true;}
    *v2pos = 2; return false;
}

// void lock(int v_id, __global int *locks) {
//   int pos = v_id % LOCK_SZ;
//   __global int * semaphor = &(locks[pos]);
//   int occupied = atom_xchg(semaphor, 1);
//   while(occupied > 0) {
//     occupied = atom_xchg(semaphor, 1);
//   }
// }

// void unlock(int v_id, __global int *locks) {
//   int pos = v_id % LOCK_SZ;
//   __global int * semaphor = &(locks[pos]);
//   atom_xchg(semaphor, 0);
// }


// void AtomicAdd(__global float *val, const float delta) {
//     union {
//         float f;
//         unsigned int i;
//     } oldVal;
//     union {
//         float f;
//         unsigned int i;
//     } newVal;

//     do {
//         oldVal.f = *val;
//         newVal.f = oldVal.f + delta;
//     } while (atom_cmpxchg((__global unsigned int *)val, oldVal.i, newVal.i) != oldVal.i);
// }
