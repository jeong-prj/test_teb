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

    void setInverse(float translation_x, float translation_y,
                    float rotation_x, float rotation_y, float rotation_z, float rotation_w);

    std::string get_frame(){return frame;};
    std::string get_child_frame(){return child_frame;};

    float get_tX(){return t_x;};
    float get_tY(){return t_y;};

    float get_rX(){return r_x;};
    float get_rY(){return r_y;};
    float get_rZ(){return r_z;};
    float get_rW(){return r_w;};

    float get_itX(){return t_xI;};
    float get_itY(){return t_yI;};

    float get_irX(){return r_xI;};
    float get_irY(){return r_yI;};
    float get_irZ(){return r_zI;};
    float get_irW(){return r_wI;};

protected:
    std::string frame;
    std::string child_frame;

    float t_x, t_y;
    float r_x, r_y, r_z, r_w;

    float t_xI, t_yI;
    float r_xI, r_yI, r_zI, r_wI;

private:


};


#endif //TEB_LOCAL_PLANNER_TRANSFORM_DATA_H
