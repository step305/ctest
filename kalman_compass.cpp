#include "kalman_compass.h"

//Hamming distance between two descriptors
// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int hammingDistance32(const int32_t *pa, const int32_t *pb)
{
    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

kalman *kalman_new() {
    kalman *state = new kalman();
    state->init();
    return state;
}

void kalman_del(kalman *kalman) {
    delete kalman;
}

void kalman::init(void) {
    P = Eigen::MatrixXf::Zero(15, 15);

    bw[0] = 0.0f; bw[1] = 0.0f; bw[2] = 0.0f;
    sw[0] = 0.0f; sw[1] = 0.0f; sw[2] = 0.0f;
    mw[0] = 0.0f; mw[1] = 0.0f; mw[2] = 0.0f;
    mw[3] = 0.0f; mw[4] = 0.0f; mw[5] = 0.0f;

    q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;
    heading = 0.0f; pitch = 0.0f; roll = 0.0f;

    cam.nRows = 480;
    cam.nCols = 640;
    cam.fc[0] = 799.28791879; //803.56099876;
    cam.fc[1] = 802.50512369; //807.13623646;
    cam.cc[0] = 323.47630502; //325.07519849;
    cam.cc[1] = 246.49945916; //245.97755414;
    cam.kc[0] = -6.83033420e-02; //7.87026530e-03;
    cam.kc[1] = 1.57869307e+00; //5.57119081e-01;
    cam.kc[2] = 7.40633413e-03; //7.75371790e-03;
    cam.kc[3] = -3.45875745e-03; //-2.16631099e-03;
    cam.kc[4] = -7.09892349e+00; //-2.67012484e+00;
    cam.frame_rate = 1.0f/15.0f;
}

void kalman::get_bias(float *bias) {
    for(int i = 0; i < 3; i++){
        bias[i] = bw[i];
    }
    return;
}

void kalman_get_bias(kalman *state, float *bias) {
    state->get_bias(bias);
    return;
}

void kalman::get_scale_errors(float *scale_errors) {
    for(int i = 0; i < 3; i++) {
        scale_errors[i] = sw[i];
    }
}

void kalman_get_scale_errors(kalman *state, float *scale_errors) {
    state->get_scale_errors(scale_errors);
    return;
}

void kalman::get_misalignment(float *misalignment) {
    for(int i = 0; i < 6; i++) {
        misalignment[i] = mw[i];
    }
}

void kalman_get_misalignment(kalman *state, float* misalignment) {
    state->get_misalignment(misalignment);
    return;
}

void kalman::get_quat(float *quat) {
    for(int i = 0; i < 4; i++) {
        quat[i] = q[i];
    }
}

void kalman_get_quat(kalman *state, float* quat) {
    state->get_quat(quat);
    return;
}

void kalman_get_angles(kalman *state, float *angles) {
    angles[2] = state->heading;
    angles[1] = state->pitch;
    angles[0] = state->roll;
}

int kalman_get_map_len(kalman *state) {
   return state->featureMap.size();
}

void kalman::fill_IMU_frame(float *dthe) {
    frame.dthe[0] = dthe[0];
    frame.dthe[1] = dthe[1];
    frame.dthe[2] = dthe[2];
    frame.points.clear();
    frame.descriptors.clear();
}

void kalman::fill_ORB_frame(int len, float *points, unsigned char *descriptors) {
    frame.points.reserve(len);
    frame.descriptors.reserve(len);

    for(int i = 0; i < 2*len; i += 2) {
        frame.points.push_back(PointsType(points[i], points[i+1]));
    }
    for(int i = 0; i < 32*len; i += 32) {
        frame.descriptors.push_back(DescriptorsType(&descriptors[i]));
    }

    //std::cout << "Reserved = " << frame.points.size() << '\n';
}

void kalman_fill_ORB_frame(kalman *state, int len, float *points, unsigned char *descriptors) {
    state->fill_ORB_frame(len, points, descriptors);
}

void kalman_get_descriptors(kalman *state, unsigned char  *descriptors) {
    for(int i=0; i < 32; i++)  {
        descriptors[i] = state->frame.descriptors[0].desc[i];
        descriptors[i+32] = state->frame.descriptors[1].desc[i];
    }
}

//SLAM Compass
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
)
{
    //Constants
    const int nxv = 15;
    const int nxf = 3;

    //Time step
    const float dt = 1.0f/100.0f;

    //DCM
    Eigen::Matrix<float,3,3> Cbn;
    Eigen::Matrix<float,3,3> Cnb;

    //Check if there's gyro measurement or camera frame present
    if (frame.points.size( ) == 0) {
        //Compensate gyro errors
        Eigen::Matrix<float,3,1> dThe{frame.dthe[0], frame.dthe[1], frame.dthe[2]};
        Eigen::Matrix<float,3,3> E;
        E << sw(0)+1.0f, mw(0), mw(1), mw(2), sw(1)+1.0f, mw(3), mw(4), mw(5), sw(2)+1.0f;
        dThe =  E*(dThe+bw*dt);

        //Attitude Quaternion Mechanization
        float gam = dThe.norm( );
        float sgam = 0.5f - powf( gam, 2.0f ) / 48.0f;
        float cgam = 1.0f - powf( gam, 2.0f ) / 8.0f + powf( gam, 4.0f ) / 384.0f;
        float lam[4];
        if (gam > 1e-16f) {
            lam[0] = cgam;
            lam[1] = -dThe(0)*sgam;
            lam[2] = -dThe(1)*sgam;
            lam[3] = -dThe(2)*sgam;
        } else {
            lam[0] = 1.0f; lam[1] = 0.0f; lam[2] = 0.0f; lam[3] = 0.0f;
        }
        float qtemp[4];
        memcpy( qtemp, q, sizeof(float)*4 );
        quat_mult( q, lam, qtemp );
        vec_normalize( (float *)q, 4 );

        Cbn << q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3],
                2.0f*(q[1]*q[2]+q[0]*q[3]),
                2.0f*(q[1]*q[3]-q[0]*q[2]),
                2.0f*(q[1]*q[2]-q[0]*q[3]),
                q[0]*q[0]-q[1]*q[1]+q[2]*q[2]-q[3]*q[3],
                2.0f*(q[2]*q[3]+q[0]*q[1]),
                2.0f*(q[1]*q[3]+q[0]*q[2]),
                2.0f*(q[2]*q[3]-q[0]*q[1]),
                q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3];
        Cnb = Cbn.transpose( );

        //Kalman Predict
        Eigen::Matrix<float,nxv,nxv> A = Eigen::MatrixXf::Zero( nxv, nxv );
        A.block<3,3>(0,3) = -Cbn;
        A.block<3,3>(0,6) = -Cbn*dThe.asDiagonal( );
        Eigen::Matrix<float,3,6> G = Eigen::MatrixXf::Zero( 3, 6 );
        G << dThe(1), dThe(2), 0.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, dThe(0), dThe(2), 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f, dThe(0), dThe(1);
        A.block<3,6>(0,9) = -Cbn*G;
        Eigen::Matrix<float,nxv,nxv> F = Eigen::MatrixXf::Identity( nxv, nxv );
        F += A * dt;
        Eigen::Matrix<float,nxv,nxv> Qn = Eigen::MatrixXf::Zero( nxv, nxv );
        float na = 1e-3f;
        float nb = 1e-7f;
        float ns = 1e-5f;
        float nm = 1e-3f;
        Eigen::Matrix<float,nxv,1> diag;
        diag << na, na, na, nb, nb, nb, ns, ns, ns, nm, nm, nm, nm, nm, nm;
        Qn.diagonal( ) = diag;
        Eigen::Matrix<float,nxv,nxv> Q = Eigen::MatrixXf::Zero( nxv, nxv );
        Q = 0.5f * dt * ( F * Qn + Qn.transpose( ) * F.transpose( ) );
        P.topLeftCorner( nxv, nxv ) = F * P.topLeftCorner( nxv, nxv ) * F.transpose( ) + Q;
        if( featureMap.size( ) > 0) {
            P.block( 0, nxv, nxv, nxf*featureMap.size( ) ) =
                    F * P.block( 0, nxv, nxv, nxf*featureMap.size( ) );
            P.block( nxv, 0, nxf*featureMap.size( ), nxv ) =
                    P.block( 0, nxv, nxv, nxf*featureMap.size( ) ).transpose( );
        }
    } else {
        //std::cout << "??\n";

        //Update State
        int nstate = nxv + featureMap.size( ) * nxf;
        Eigen::Matrix<float, Eigen::Dynamic, 1> X( nstate, 1 );
        X = Eigen::MatrixXf::Zero( nstate, 1 );

        //Match counter
        const int min_match = 15;
        const int max_match = 30;
        int match_cnt = 0;

        //Containers for feature augmentation
        std::vector<Eigen::Matrix<float,3,1>> en_augment;
        std::vector<std::vector<int32_t>> ds_augment;
        en_augment.reserve(50);
        ds_augment.reserve(50);

        //Kalman matrices
        Eigen::Matrix<float,Eigen::Dynamic,3> K( nstate, nxf );
        Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> KH( nstate, nstate );
        Eigen::Matrix<float,nxf,nxf> S;
        Eigen::Matrix<float,3,1> V;
        Eigen::Matrix<float,nxf,nxf> R = Eigen::MatrixXf::Identity( nxf, nxf );
        R *= 0.005f;
        Eigen::Matrix<float,nxf,nxf> Cbneb = Eigen::MatrixXf::Zero( nxf, nxf );
        Eigen::Matrix<float,nxf,nxf> Sinv;
        Eigen::Matrix<float,3,1> en_map;
        Eigen::Matrix<float,3,1> eb_map;
        Cbn << q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3],
                2.0f*(q[1]*q[2]+q[0]*q[3]),
                2.0f*(q[1]*q[3]-q[0]*q[2]),
                2.0f*(q[1]*q[2]-q[0]*q[3]),
                q[0]*q[0]-q[1]*q[1]+q[2]*q[2]-q[3]*q[3],
                2.0f*(q[2]*q[3]+q[0]*q[1]),
                2.0f*(q[1]*q[3]+q[0]*q[2]),
                2.0f*(q[2]*q[3]-q[0]*q[1]),
                q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3];
        Cnb = Cbn.transpose( );

        //Iterate through measured features
        for( int meas = 0; meas<frame.points.size( ); meas++ ) {
            //Measurement unit vector in body frame
            bool valid = true;
            Eigen::Matrix<float,3,1> eb_mes = {
                    sqrtf( 1.0f-( powf( frame.points[meas].x, 2.0f ) +
                                  powf( frame.points[meas].y, 2.0f ) ) ),
                    frame.points[meas].x,
                    frame.points[meas].y };
            //std::cout << eb_mes(0) << ' ' << Cbn << " point\n";


            //Measurement  unit vector in navigation frame
            Eigen::Matrix<float,3,1> en_mes = Cbn * eb_mes;


            //Map management flag
            bool close = false;

            //Look for the matching feature on the map
            for( auto &feature : featureMap ) {
                //b-frame unit vector for map feature
                en_map = feature.en;

                //Map feature unit vector in b-frame
                //Project map feature to the camera matrix
                //Memoization
                float u, v;
                if ( !feature.vis ) {
                    //Map feature unit vector in b-frame
                    eb_map = Cnb * en_map;
                    feature.eb = eb_map;
                    //Project map feature to the camera matrix
                    float r2 = eb_map(1)*eb_map(1)+eb_map(2)*eb_map(2);
                    float r4 = powf( r2, 2.0 );
                    float r6 = powf( r2, 3.0 );
                    float xd = eb_map(1)*(1+cam.kc[0]*r2+cam.kc[1]*r4+cam.kc[4]*r6) +
                               2*cam.kc[2]*eb_map(1)*eb_map(2)+cam.kc[3]*(r2+2*eb_map(1)*eb_map(1));
                    float yd = eb_map(2)*(1+cam.kc[0]*r2+cam.kc[1]*r4+cam.kc[4]*r6) +
                               cam.kc[2]*(r2+2*eb_map(2)*eb_map(2))+2*cam.kc[3]*eb_map(1)*eb_map(2);
                    u =  cam.fc[0] * xd + cam.cc[0];
                    v =  cam.fc[1] * yd + cam.cc[1];
                    feature.u = u;
                    feature.v = v;
                    feature.vis = true;
                } else {
                    eb_map = feature.eb;
                    u = feature.u;
                    v = feature.v;
                }

                //If map feature fits to camera and doesn't fall to the excluded band
                float excluded_band = 10.0;

                if ( feature.obs || ( ( u > excluded_band ) && (u < (cam.nCols-excluded_band) ) &&
                                      (v > excluded_band) && (v < (cam.nRows-excluded_band) ) &&
                                      (eb_map(0) > 0.9 ) ) ) {

                    //Feature observed
                    feature.obs = true;

                    //Check if the measured feature is close to the one on the map
                    //Feature and measurement angles
                    float theta_f = atan2f( en_map( 1 ), en_map( 0 ) );
                    float theta_m = atan2f( en_mes( 1 ), en_mes( 0 ) );
                    float phi_f = atan2f( en_map( 2 ),
                                          sqrtf( en_map(0)*en_map(0) + en_map(1)*en_map(1) ) );
                    float phi_m = atan2f( en_mes( 2 ),
                                          sqrtf( en_mes(0)*en_mes(0) + en_mes(1)*en_mes(1) ) );
                    if ( ( fabsf(theta_f-theta_m) < 0.1f ) &&
                         ( fabsf(phi_f-phi_m) < 0.1f ) )
                        close = true;
                    //std::cout << "feature close = " << X(0,0) << ' ' << X(3,0) << '\n';

                    if ( match_cnt < max_match ) {
                        //Measurement ORB Descriptor
                        int32_t des_mes[8];
                        get_descriptor32(frame.descriptors[meas].desc, des_mes);
                        //std::cout << "bias maxmatch = " << X(0,0) << ' ' << X(3,0) << '\n';

                        //Calculate Hamming distance between ORB descriptors
                        //(number of differing bits in binary representation)
                        //If map and measurement features match

                        const int hamming_threshold = 20;
                        if ( hammingDistance32( des_mes, feature.des ) < hamming_threshold ) {
                            //std::cout << "bias Hamming = " << X(0,0) << ' ' << X(3,0) << '\n';
                            //Measurement vector and matrix
                            V = eb_mes - eb_map;
                            Eigen::Matrix<float,3,3>  Hv;
                            Eigen::Matrix<float,3,3>  Hf;
                            Cbneb(0,1) = -en_mes(2);
                            Cbneb(0,2) =  en_mes(1);
                            Cbneb(1,0) =  en_mes(2);
                            Cbneb(1,2) = -en_mes(0);
                            Cbneb(2,0) = -en_mes(1);
                            Cbneb(2,1) =  en_mes(0);
                            Hv = -Cnb*Cbneb;
                            Hf =  Cnb;

                            //Innovation Covariance
                            Eigen::Matrix<float,3,3> Pvv = P.topLeftCorner( 3, 3 );
                            Eigen::Matrix<float,3,3> Pvf = P.block<3,3>(0,feature.pos);
                            Eigen::Matrix<float,3,3> Pfv = P.block<3,3>(feature.pos,0);
                            Eigen::Matrix<float,3,3> Pff = P.block<3,3>(feature.pos,feature.pos);
                            S = ( Hv * Pvv + Hf * Pfv ) * Hv.transpose( ) + ( Hv * Pvf + Hf * Pff ) * Hf.transpose( ) + R;
                            Sinv = S.inverse( );

                            //Check Chi2 threshold
                            float chi2 = V.transpose( ) * Sinv * V;

                            //If threshold was met
                            if ( chi2 < 6.25 ) {
                                //Matched   feature
                                feature.mat = true;
                                match_cnt++;

                                //Kalman Gain
                                int pos = 0;
                                K = Eigen::MatrixXf::Zero( nstate, nxf );
                                KH = Eigen::MatrixXf::Zero( nstate, nstate );
                                while( pos < nstate ) {
                                    K.block( pos, 0, 3, 3 ) = (P.block( pos, 0, 3, 3 ) * Hv.transpose( ) +
                                                               P.block( pos, feature.pos, 3, 3 ) * Hf.transpose( ) ) * Sinv;
                                    KH.block( pos, 0, 3, 3 ) = -K.block( pos, 0, 3, 3 ) * Hv;
                                    KH.block( pos, feature.pos, 3, 3 ) = -K.block( pos, 0, 3, 3 ) * Hf;
                                    pos += 3;
                                }
                                for (int k=0; k<nstate; ++k)
                                    KH(k,k) += 1.0f;

                                //Update Covariance

                                auto PP = KH * P * KH.transpose( ) + K * R * K.transpose( );
                                P = PP;

                                //Update State
                                X += K*V;
                                //std::cout << "bias XX= " << X(0,0) << ' ' << X(3,0) << '\n';

                                //Make P symmetric
                                auto PP2 = 0.5 * ( P + P.transpose( ) );
                                P = PP2;
                                break;
                            }
                        }// if( hamming ...
                    }// if (nrm < prox_threshold)
                } // if(( u > ...
                else {
                    valid = false;
                }
            }//for( feature_map

            //Data for feature augmentation
            if( !close && valid) {
                const int32_t* p = (int32_t*)frame.descriptors[meas].desc;
                std::vector<int32_t> des(p, p + 8);
                en_augment.push_back(en_mes);
                ds_augment.push_back(des);
            }

        }//for( keypoints
        //std::cout << "match cnt " << match_cnt << ' ' << featureMap.size() << ' ' << uuu << '\n';

        //Correct Attitude Quaternion
        float qe[4];
        qe[1] = X(0)* 0.5f;
        qe[2] = X(1) * 0.5f;
        qe[3] = X(2) * 0.5f;
        float mgn = sqrtf( qe[1] * qe[1] + qe[2] * qe[2] + qe[3] * qe[3] );
        if( mgn < 1.0f ) {
            qe[0] = sqrtf( 1.0f - mgn );
            float qtemp[4];
            quat_mult( qtemp, q, qe );
            memcpy( q, qtemp, sizeof(qtemp) );
        }

        //std::cout << "C: " << X[3, 0] << ' ' << X[4,0] << ' ' << X[5, 0] << '\n';

        //Update gyro bias
        bw(0) = bw(0) + X(3);
        bw(1) = bw(1) + X(4);
        bw(2) = bw(2) + X(5);

        //Update gyro scale
        sw(0) = sw(0) + X(6);
        sw(1) = sw(1) + X(7);
        sw(2) = sw(2) + X(8);

        //Update gyro misalignment
        mw(0) = mw(0) + X(9);
        mw(1) = mw(1) + X(10);
        mw(2) = mw(2) + X(11);
        mw(3) = mw(3) + X(12);
        mw(4) = mw(4) + X(13);
        mw(5) = mw(5) + X(14);

        //Correct Map features n-frame unit vectors
        for( auto &feature : featureMap ) {
            feature.en += X.block( feature.pos, 0, nxf, 1 );
            //feature.en(0) = feature.en(0) + X(feature.pos, 0);
            //feature.en(1) = feature.en(1) + X(feature.pos+1, 0);
            //feature.en(2) = feature.en(2) + X(feature.pos+2, 0);
            //feature.cnt_obs += feature.obs;
            feature.cnt_mat += feature.mat;
        }

        //MAP MANAGEMENT: Delete faulty features
        Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> P_( nstate, nstate );
        P_ = P;
        std::vector<int> index_keep{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14 }; //always keep attitude and gyro state
        index_keep.reserve(nstate);
        int pos = 15;
        auto it = featureMap.begin( );
        while( it != featureMap.end( ) ) {
            float obs_rate = (float)((*it).cnt_mat) / (float)( (*it).cnt_obs );
            float obs_thr = 0.2f;
            if ( obs_rate < obs_thr ) {
                erased.push_back( PointsType( (*it).u, (*it).v ) );
                it = featureMap.erase(it);
            } else {
                index_keep.push_back((*it).pos);
                index_keep.push_back((*it).pos+1);
                index_keep.push_back((*it).pos+2);
                (*it).pos = pos;
                pos += 3;
                ++it;
            }
        }
        nstate = nxv + featureMap.size( ) * nxf;
        Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> P_new( nstate, nstate );
        P_new = P(index_keep, index_keep);
        P.resize( nstate, nstate );
        P = P_new;

        //MAP MANAGEMENT: Add new features
        //std::cout << "map len = " << featureMap.size() << '\n';
        const int max_map = 200;
        if( match_cnt < min_match) {
            int naugm = min_match - match_cnt;
            naugm = ( naugm >= en_augment.size( ) ) ? en_augment.size( ) : naugm;
            for( int i = 0; ( ( i < naugm ) && ( featureMap.size( ) < max_map ) ) ; ++i ) {

                //New feature
                FeatureStruct feature;

                //Flags and counters
                feature.cnt_obs = 0;
                feature.cnt_mat = 0;
                feature.mat = false;
                feature.obs = false;
                feature.vis = false;

                //Feature unit vector
                Eigen::Matrix<float,3,1> en = en_augment[i];
                feature.en = en;

                //Feature ORB Descriptor
                std::vector<int32_t> des = ds_augment[i];
                for( int j=0; j<des.size( ); ++j )
                    feature.des[j] = des[j];

                //Covariance matrix rows and columns indices for the feature
                feature.pos = ( nxv + nxf * ( featureMap.size( ) + 1 ) ) - 3;

                //Add feature to the Map
                featureMap.push_back( feature );

                //Augment Covariance Matrix
                Eigen::Matrix<float,nxf,nxv> Hv = Eigen::MatrixXf::Zero( nxf, nxv );
                Hv(0,1) = -en(2);
                Hv(0,2) =  en(1);
                Hv(1,0) =  en(2);
                Hv(1,2) = -en(0);
                Hv(2,0) = -en(1);
                Hv(2,1) =  en(0);
                Eigen::Matrix<float,nxf,nxf> Hz = Cbn;
                Eigen::Matrix<float,nxf,nxf> R = Eigen::MatrixXf::Identity( nxf, nxf );
                R *= 0.03f;

                //Add rows and columns to Covariance Matrix
                Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> P_new( P.rows( ) + nxf, P.cols( ) + nxf );
                P_new = Eigen::MatrixXf::Zero( P.rows( ) + nxf, P.cols( ) + nxf );
                P_new.topLeftCorner( P.rows( ), P.cols( ) ) = P;
                P.resize( P_new.rows( ), P_new.cols( ) );
                P = P_new;

                //Feature covariance
                P.block( feature.pos, feature.pos, nxf, nxf ) =
                        Hv * P.topLeftCorner( nxv, nxv ) * Hv.transpose( ) +
                        Hz * R * Hz.transpose( );

                //Vehicle to feature cross-covariance
                P.block( feature.pos, 0, nxf, nxv ) =
                        Hv * P.topLeftCorner(nxv, nxv );
                P.block( 0, feature.pos, nxv, nxf ) =
                        P.block( feature.pos, 0, nxf, nxv ).transpose( );

                //Feature to feature cross-covariance
                auto start = featureMap.begin( );
                auto stop = std::prev(featureMap.end( ) );
                for( auto it = start; it != stop; ++it ) {
                    P.block( feature.pos, (*it).pos, nxf, nxf ) =
                            Hv * P.block( 0, (*it).pos, nxv, nxf );
                    P.block( (*it).pos, feature.pos, nxf, nxf ) =
                            P.block( feature.pos, (*it).pos, nxf, nxf ).transpose( );
                }
            } //for augment
        } // match_cnt < min_match
        //std::cout << "map len = " << featureMap.size() << '\n';
    } //if
}// Compass

void kalman_run_compass_imu(kalman *state, float *dthe) {
    state->fill_IMU_frame(dthe);
    std::vector<PointsType> erased_points;
    float quat[4];
    quat[0] = state->q[0];
    quat[1] = state->q[1];
    quat[2] = state->q[2];
    quat[3] = state->q[3];
    Compass(state->featureMap, state->P, quat, state->bw, state->sw, state->mw,
            state->frame, state->cam, erased_points);
    //std::cout << "map len = " << state->featureMap.size() << '\n';
    float qc[4];
    float r[3];
    memcpy( qc, quat, sizeof(quat) );
    quatconj( qc );
    quat2angle( qc, r );
    state->q[0] = quat[0];
    state->q[1] = quat[1];
    state->q[2] = quat[2];
    state->q[3] = quat[3];
    state->heading = r[2]*180.0f/3.14159265f;
    state->pitch   = r[1]*180.0f/3.14159265f;
    state->roll    = r[0]*180.0f/3.14159265f;
}

void kalman_run_compass_orb(kalman *state, int len, float *points, unsigned char *descriptors) {
    state->fill_ORB_frame(len, points, descriptors);
    std::vector<PointsType> erased_points;
    float quat[4];
    quat[0] = state->q[0];
    quat[1] = state->q[1];
    quat[2] = state->q[2];
    quat[3] = state->q[3];
   // std::cout << state->bw << '\n';
    Compass(state->featureMap, state->P, quat, state->bw, state->sw, state->mw,
            state->frame, state->cam, erased_points);
    float qc[4];
    float r[3];
    memcpy( qc, quat, sizeof(quat) );
    quatconj( qc );
    quat2angle( qc, r );
    state->q[0] = quat[0];
    state->q[1] = quat[1];
    state->q[2] = quat[2];
    state->q[3] = quat[3];
    state->heading = r[2]*180.0f/3.14159265f;
    state->pitch   = r[1]*180.0f/3.14159265f;
    state->roll    = r[0]*180.0f/3.14159265f;

    int match_cnt = 0;
    float u, v;
    std::vector<PointsType> matched_points;
    std::vector<PointsType> all_points;
    matched_points.reserve(50);
    all_points.reserve(100);
    //Features
    if (state->frame.points.size() > 0) {
        for (auto &feature : state->featureMap) {
            u = feature.u;
            v = feature.v;
            if( feature.mat ) {
                match_cnt++;
                matched_points.push_back( PointsType( u, v ) );
            } else if ( feature.obs )
                all_points.push_back( PointsType( u, v ) );
            feature.mat = false;
            feature.obs = false;
            feature.vis = false;
        }
    }
}
