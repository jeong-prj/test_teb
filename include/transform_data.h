//
// Created by ej on 23. 8. 21.
//

#ifndef TEB_LOCAL_PLANNER_TRANSFORM_DATA_H
#define TEB_LOCAL_PLANNER_TRANSFORM_DATA_H

#include <iostream>

class TfData {

public:
    TfData();
    virtual ~TfData();

    void initialize(std::string p_frame, std::string c_frame,
                    float translation_x, float translation_y,
                    float rotation_x, float rotation_y, float rotation_z, float rotation_w);

    std::string get_frame(){return frame;};
    std::string get_child_frame(){return child_frame;};

    float get_tX(){return t_x;};
    float get_tY(){return t_y;};

    float get_rX(){return r_x;};
    float get_rY(){return r_y;};
    float get_rZ(){return r_z;};
    float get_rW(){return r_w;};

protected:
    std::string frame;
    std::string child_frame;

    float t_x;
    float t_y;
//    float t_z;

    float r_x;
    float r_y;
    float r_z;
    float r_w;

private:


};


#endif //TEB_LOCAL_PLANNER_TRANSFORM_DATA_H
