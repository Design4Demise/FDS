#include "../include/gust_model.h"

DrydenGustModel::DrydenGustModel(WindClass &wind) {

    windPtr = &wind;
    windPtr->gustPtr = this;

    updated_vel = 0;

}

void DrydenGustModel::update_tf() {

    double v_air = windPtr->dynamicsPtr->v_air;

    // define numerators
    double a1 = sigma_u * std::sqrt(2.0 * v_air / (M_PI * lu));

    double a2 = sigma_v * std::sqrt(3.0 * v_air / (M_PI * lv));
    double a3 = a2 * v_air / (std::sqrt(3.0) * lv);

    double a4 = sigma_w * std::sqrt(3.0 * v_air / (M_PI * lw));
    double a5 = a4 * v_air / (std::sqrt(3.0) * lw);

    // Eigen::RowVectorXd num_h_u, num_h_v, num_h_w;
    Eigen::Matrix<double, 1, 1> num_h_u(a1);
    Eigen::RowVector2d num_h_v(a2, a3);
    Eigen::RowVector2d num_h_w(a4, a5);

    // define denominators
    double b1 = v_air / lu;
    double b2 = v_air / lv;
    double b3 = v_air / lw;

    // Eigen::RowVectorXd den_h_u, den_h_v, den_h_w;
    Eigen::RowVector2d den_h_u(1.0, b1);
    Eigen::RowVector3d den_h_v(1.0, 2.0 * b2, SQ(b2));
    Eigen::RowVector3d den_h_w(1.0, 2.0 * b3, SQ(b3));

    // define transfer functions
    h_u = TransferFunction(num_h_u, den_h_u, time_step);
    h_v = TransferFunction(num_h_v, den_h_v, time_step);
    h_w = TransferFunction(num_h_w, den_h_w, time_step);

}

void DrydenGustModel::update_gust() {

    if (!updated_vel) {
        update_tf();
        updated_vel = true;
    }

    gust(eu) = h_u.update(dist(generator));
    gust(ev) = h_v.update(dist(generator));
    gust(ew) = h_w.update(dist(generator));

    windPtr->windState(Eigen::seq(ew_gust_u, ew_gust_w)) = gust;

}
