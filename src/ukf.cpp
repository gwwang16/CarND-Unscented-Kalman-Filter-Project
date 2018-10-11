#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.6;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.2;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  //set initialize state
  is_initialized_ = false;
  //set state dimension
  n_x_ = 5;
  //set augmented dimension
  n_aug_ = 7;
  //define spreading prameter
  lambda_ = 3 - n_aug_;

  //set initial nis value
  NIS_lidar_ = 0;
  NIS_radar_ = 0;

  //set vector for weights
  weights_ = VectorXd(2*n_aug_+1);
  // set weights
  weights_(0) = lambda_/(lambda_+n_aug_);
  for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights
    weights_(i) = 0.5/(n_aug_+lambda_);
  }

  //predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);

  //start time
  time_us_ = 0;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_){
    //initialie x_ stateof CTRV model: px, py, v, vx, vy.
    float v_initial = 5.2;
    x_ << 1, 1, v_initial, 0, 0;
    //initialize covariance matrix
    P_ << 0.00371256, 0.000766989,  0.00412177, 0.000307604, 0.000301292,
          0.000766989,  0.00425996,  0.00123312,  0.00148717,  0.00145772,
          0.00412177,  0.00123312,  0.00987088, 0.000624762, 0.000661055,
          0.000307604,  0.00148717, 0.000624762,   0.0011231,  0.00186524,
          0.000301292,  0.00145772, 0.000661055,  0.00186524,  0.00525917;

    if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) & use_radar_){
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      // rho, phi, rho_dot
      float rho = meas_package.raw_measurements_(0);
      float phi = meas_package.raw_measurements_(1);
      x_(0) = rho * cos(phi);
      x_(1) = rho * sin(phi);
    }
    else if ((meas_package.sensor_type_ == MeasurementPackage::LASER) & use_laser_) {
      // px, py
      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(1);
    }

    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;

    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  double delta_t = (meas_package.timestamp_ - time_us_)/1000000.0;
  time_us_ = meas_package.timestamp_;

  Prediction(delta_t);
  

  /*****************************************************************************
   *  Update
  ****************************************************************************/
  if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) & use_radar_) {
    // Radar updates
    UpdateRadar(meas_package);
  } else if ((meas_package.sensor_type_ == MeasurementPackage::LASER) & use_laser_){
    // Laser updates
    UpdateLidar(meas_package);
  }

  // print the output
  cout << "x_ = " << x_.transpose() << endl;
  cout << "P_ = \n" << P_ << endl;
  cout << "NIS_lidar: " << NIS_lidar_ << '\t';
  cout << "NIS_radar: " << NIS_radar_ << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);

  /*****generate sigma points*****/

  //create augmented mean state
  x_aug.head(5) = x_;
  //the mean of the process noise is zero.
  x_aug(5) = 0;
  x_aug(6) = 0;

  P_aug.setZero();
  P_aug.topLeftCorner(P_.rows(), P_.cols()) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  //Cholesky Decomposition to calculate sqrt of matrix
  MatrixXd L = P_aug.llt().matrixL();

  Xsig_aug.col(0) = x_aug;
  for(int i=0; i<n_aug_; i++){
    Xsig_aug.col(i+1) = x_aug + sqrt(lambda_+n_aug_)*L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_)*L.col(i);
  }

  /*****predict sigma points*****/

  for (int i = 0; i<2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * (cos(yaw) - cos(yaw+yawd*delta_t));
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  /*****predict mean and covariance*****/

  x_.setZero();
  // set weights
  weights_(0) = lambda_/(lambda_+n_aug_);
  for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights
    double weight = 0.5/(n_aug_+lambda_);
    weights_(i) = weight;
  }

  //predicted state mean
  for (int i = 0; i <2*n_aug_+1; i++) {  //iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  P_.setZero();
  //predicted state covariance matrix
  for (int i = 0; i<2*n_aug_+1; i++) {  //iterate over sigma points
    // state difference
    // VectorXd x_diff = Xsig_pred_.col(i) - x_;
    VectorXd x_diff = Xsig_pred_.col(i) - Xsig_pred_.col(0);
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3) -= 2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3) += 2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  // KF for linear Lidar measurement
  //set measurement dimension, Liadar can measure px and py
  int n_z = 2;

  //measurement covariance noise
  MatrixXd R = MatrixXd(n_z, n_z);
  //add measurement noise covariance matrix
  R << std_laspx_*std_laspx_, 0,
       0, std_laspy_*std_laspy_;

  //extract lidar measurement
  VectorXd z = meas_package.raw_measurements_;

  MatrixXd H = MatrixXd(n_z, n_x_);
  H << 1, 0, 0, 0, 0,
       0, 1, 0, 0, 0;

  VectorXd z_pred = H * x_;

  VectorXd y = z - z_pred;

  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P_ * Ht + R;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  // estimate
  x_ = x_ + K*y;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H) * P_;

  NIS_lidar_ = y.transpose() * Si * y;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  // EKF for Radar measurement

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  VectorXd z = meas_package.raw_measurements_;

  //create sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  //calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //measurement covariance noise
  MatrixXd R = MatrixXd(n_z, n_z);

  //transform predict sigma points into measurement space
  for (int i=0; i<2*n_aug_+1; i++){
    double px = Xsig_pred_(0,i);
    double py = Xsig_pred_(1,i);
    double v = Xsig_pred_(2,i);
    double psi = Xsig_pred_(3,i);
    // double psi_d = Xsig_pred_(4,i);
    //measurement model
    double rho = sqrt(px*px + py*py);
    double phi = atan2(py, px);
    double rho_d = (px*cos(psi)*v + py*sin(psi)*v)/rho;
    Zsig.col(i) << rho, phi, rho_d;
  }

  //mean predicted measurement
  z_pred.setZero();
  for (int i=0; i<2*n_aug_+1; i++){
    z_pred += weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix
  S.setZero();
  for (int i=0; i<2*n_aug_+1; i++){
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while(z_diff(1)>M_PI) z_diff(1) -= 2*M_PI;
    while(z_diff(1)<-M_PI) z_diff(1) += 2*M_PI;
    S += weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  
  R << std_radr_*std_radr_, 0, 0,
       0, std_radphi_*std_radphi_, 0,
       0, 0, std_radrd_*std_radrd_;
  S += R;

  /* update state
  */
  Tc.setZero();
  for (int i=0;i<2*n_aug_+1; i++){
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  MatrixXd Si = S.inverse();
  MatrixXd K = Tc * Si;

  x_ = x_ + K*(z - z_pred);

  P_ = P_ - K * S * K.transpose();

  VectorXd z_diff(n_z);
  z_diff  = z - z_pred;
  NIS_radar_ = z_diff.transpose() * Si * z_diff;
}
