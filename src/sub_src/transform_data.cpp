//
// Created by ej on 23. 8. 21.
//

#include "transform_data.h"
/*
TfData::TfData(){

}

TfData::~TfData(){

}*/
void TfData::initialize(std::string p_frame, std::string c_frame,
                        float translation_x, float translation_y,
                        float rotation_x, float rotation_y, float rotation_z, float rotation_w){
    frame = p_frame;
    child_frame = c_frame;

    t_x = translation_x;
    t_y = translation_y;

    r_x = rotation_x;
    r_y = rotation_y;
    r_z = rotation_z;
    r_w = rotation_w;
}

