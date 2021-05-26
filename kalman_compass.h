#ifndef INC_KALMAN_COMPASS_H
#define INC_KALMAN_COMPASS_H

#include <stdio.h>
#include <iostream>
#include <atomic>
#include <chrono>
#include <thread>
#include <stdlib.h>
#include <numeric>
#include <time.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <semaphore.h>
#include <iterator>
#include <unordered_map>
#include <list>
#include "Eigen/Dense"
#include "attitude_mechanization.h"
#include "transformations.h"
#include "utils.hpp"

typedef struct DescriptorsType {
    unsigned char desc[32];
    DescriptorsType(unsigned char d[32]) {
        for(int i = 0; i < 32; i++) {
            desc[i] = d[i];
        }
    }
} DescriptorsType;

typedef struct PointsType {
    float x;
    float y;
    PointsType(float u, float v) {
        x = u;
        y = v;
    }
}PointsType;

typedef struct SLAMMessageStruct {
public:
    long long unsigned ts;
    std::array<float,3> dthe;
    std::vector<DescriptorsType> descriptors;
    std::vector<PointsType>  points;
} SLAMMessageStruct;

typedef struct {
    int nRows;
    int nCols;
    float fc[2];
    float cc[2];
    float kc[5];
    float frame_rate;
} CamStruct;

typedef struct {
    Eigen::Matrix<float,3,1> en; //unit vector in navigation frame
    Eigen::Matrix<float,3,1> eb; //unit vector in navigation frame
    float u;
    float v;
    int32_t des[8]; //ORB descriptor
    int pos; //feature starting position in state vector
    bool obs; //observed flag
    bool mat; //matched flag
    bool vis; //visited flag
    int cnt_obs;
    int cnt_mat;
} FeatureStruct;

using MapType = std::list<FeatureStruct>;

void Compass(
        MapType &featureMap,
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &P,
        float q[4],
        Eigen::Matrix<float,3,1> &bw,
        Eigen::Matrix<float,3,1> &sw,
        Eigen::Matrix<float,6,1> &mw,
        SLAMMessageStruct &frame,
        const CamStruct &cam,
        std::vector<PointsType>  &erased
);

class kalman {
public:
    CamStruct cam;
    int nxv;
    Eigen::Matrix<float,3,1> bw;
    Eigen::Matrix<float,3,1> sw;
    Eigen::Matrix<float,6,1> mw;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> P;
    MapType featureMap;
    float heading;
    float pitch;
    float roll;
    float q[4];
    SLAMMessageStruct frame;

    void init(void);
    void get_bias(float *bias);
    void get_scale_errors(float *scale_errors);
    void get_misalignment(float *misalignment);
    void get_quat(float *quat);
    void fill_IMU_frame(float* dthe);
    void fill_ORB_frame(int len, float* points, unsigned char *descriptors);
};

extern "C" {
    kalman *kalman_new();
    void kalman_del(kalman *kalman);
    void kalman_get_bias(kalman *state, float* bias);
    void kalman_get_scale_errors(kalman *state, float* scale_errors);
    void kalman_get_misalignment(kalman *state, float* misalignment);
    void kalman_get_quat(kalman *state, float* quat);
    int kalman_get_map_len(kalman *state);
    void kalman_get_angles(kalman *state, float *angles);
    void kalman_run_compass_imu(kalman *state, float *dthe);
    void kalman_run_compass_orb(kalman *state, int len, float *points, unsigned char *descriptors);
    void kalman_get_descriptors(kalman *state, unsigned char  *descriptors);
    void kalman_fill_ORB_frame(kalman *state, int len, float *points, unsigned char *descriptors);
}

#endif //INC_KALMAN_COMPASS_H
