//
// Created by Tiezheng YU on 2019/2/25.
//

#include "Params.h"

//--------- Camera-------------------------------------------------------------------------
float Params::camera_width = 752;
float Params::camera_height = 480;
float Params::camera_fps = 30;
float Params::camera_fu = 500;
float Params::camera_fv = 500;
float Params::camera_cu = 320;
float Params::camera_cv = 240;
float Params::camera_k1 = 0;
float Params::camera_k2 = 0;
float Params::camera_p1 = 0;
float Params::camera_p2 = 0;
float Params::distortion_coefficients[4] = {0,0,0,0};
float Params::camera_extrinsics[4][4] = 
        {0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
         0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
        -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
         0.0, 0.0, 0.0, 1.0};
//---------Ground Truth--------------------------------------------------------------------
float Params::p_RS_R_x = 0.878612;
float Params::p_RS_R_y = 2.142470;
float Params::p_RS_R_z = 0.947262;
float Params::q_RS_w = 0.060514;
float Params::q_RS_x = -0.828459;
float Params::q_RS_y = -0.058956;
float Params::q_RS_z = -0.553641;
//---------Point Cloud Params-----------------------------------------------
float Params::max_depth = 0;