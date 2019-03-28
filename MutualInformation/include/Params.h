//
// Created by Tiezheng YU on 2019/2/25.
//

#ifndef HSVDETECT_MARKERPARAMS_H
#define HSVDETECT_MARKERPARAMS_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

using namespace std;
class Params {
public:
//---------camera params--------------------------------------------------
    static float camera_width;
    static float camera_height;
    static float camera_fps;
    static float camera_fu;
    static float camera_fv;
    static float camera_cu;
    static float camera_cv;
    static float camera_k1;
    static float camera_k2;
    static float camera_p1;
    static float camera_p2;
    static float camera_extrinsics[4][4];
    static float distortion_coefficients[4];
//--------- Ground truth----------------------------------------------------
    static float p_RS_R_x;
    static float p_RS_R_y;
    static float p_RS_R_z;
    static float q_RS_w;
    static float q_RS_x;
    static float q_RS_y;
    static float q_RS_z;
//---------Point Cloud Params-----------------------------------------------
    static float max_depth;

//--------- using for Grandient Algrathim-----------------------------------
    
};


#endif //HSVDETECT_MARKERPARAMS_H
