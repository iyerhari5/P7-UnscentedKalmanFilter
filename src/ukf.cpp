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
  P_ << 1, 0, 0, 0, 0,
	  0, 1, 0, 0, 0,
	  0, 0, 1, 0, 0,
	  0, 0, 0, 1, 0,
	  0, 0, 0, 0, 1;


  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.7;
  
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
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  //set state dimension
  n_x_ = 5;

  //set augmented dimension
  n_aug_ = 7;

  //define spreading parameter
  lambda_ = 3 - n_aug_;

  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  //create vector for weights
  weights_ = VectorXd(2 * n_aug_ + 1);

  //set weights
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i<2 * n_aug_ + 1; i++)
	  weights_(i) = 0.5 / (lambda_ + n_aug_);

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
	if (!is_initialized_) {
		cout << "UKF: initializing" << endl;

		if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

			//do nothing
			//if (!use_radar_)
			//	return;

			/**
			Convert radar from polar to cartesian coordinates and initialize state.
			*/
			x_[0] = meas_package.raw_measurements_[0] * cos(meas_package.raw_measurements_[1]);
			x_[1] = meas_package.raw_measurements_[0] * sin(meas_package.raw_measurements_[1]);
			x_[2] = 0;
			x_[3] = 0;
			x_[4] = 0;
		}
		else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
			/**
			Initialize state.
			*/

			//do nothing
			//if (!use_laser_)
			//	return;

			x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;

		}

		//Get the current timestamp
		previous_timestamp_ = meas_package.timestamp_;

		// done initializing, no need to predict or update
		is_initialized_ = true;
		return;
	}

	/*****************************************************************************
	*  Prediction
	****************************************************************************/

	//do nothing
	if (!use_radar_ && (meas_package.sensor_type_ == MeasurementPackage::RADAR))
		return;

	if (!use_laser_ && (meas_package.sensor_type_ == MeasurementPackage::LASER))
		return;

	//compute the time elapsed between the current and previous measurements
	float dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
	previous_timestamp_ = meas_package.timestamp_;

	Prediction(dt);

	if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
		UpdateRadar(meas_package);
	
	if (meas_package.sensor_type_ == MeasurementPackage::LASER)
		UpdateLidar(meas_package);

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
	MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

	//create augmented mean state
	x_aug.setZero();
	x_aug.head(n_x_) = x_;

	//create augmented covariance matrix
	P_aug.setZero();
	P_aug.topLeftCorner(n_x_, n_x_) = P_;
	P_aug(n_x_, n_x_) = std_a_*std_a_;
	P_aug(n_x_ + 1, n_x_+ 1) = std_yawdd_*std_yawdd_;

	//calculate square root of P_aug
	MatrixXd A = P_aug.llt().matrixL();

	//create augmented sigma points
	//set first column of sigma point matrix
	Xsig_aug.col(0) = x_aug;

	//set remaining sigma points
	for (int i = 0; i < n_aug_; i++)
	{
		Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * A.col(i);
		Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * A.col(i);
	}


	//Sigmapoint prediction
	float px;
	float py;
	float v;
	float psi;
	float psiDot;
	float nuA;
	float nuPsiDotDot;
	float sinPsi;
	float cosPsi;


	//predict sigma points
	//avoid division by zero
	//write predicted sigma points into right column
	for (int i = 0; i< 2 * n_aug_ + 1; i++)
	{

		//std::cout << i << std::endl;

		px = Xsig_aug(0, i);
		py = Xsig_aug(1, i);
		v = Xsig_aug(2, i);
		psi = Xsig_aug(3, i);
		psiDot = Xsig_aug(4, i);
		nuA = Xsig_aug(5, i);
		nuPsiDotDot = Xsig_aug(6, i);
		sinPsi = sin(psi);
		cosPsi = cos(psi);

		if (psiDot != 0)
		{
			Xsig_pred_(0, i) = px + v / psiDot*(sin(psi + psiDot*delta_t) - sinPsi) + 0.5*delta_t*delta_t*cosPsi*nuA;
			Xsig_pred_(1, i) = py + v / psiDot*(-cos(psi + psiDot*delta_t) + cosPsi) + 0.5*delta_t*delta_t*sinPsi*nuA;
		}
		else
		{
			Xsig_pred_(0, i) = px + v*cosPsi*delta_t + 0.5*delta_t*delta_t*cosPsi*nuA;
			Xsig_pred_(1, i) = py + v*sinPsi*delta_t + 0.5*delta_t*delta_t*sinPsi*nuA;
		}

		Xsig_pred_(2, i) = v + delta_t*nuA;
		Xsig_pred_(3, i) = psi + psiDot*delta_t + 0.5*delta_t*delta_t*nuPsiDotDot;
		Xsig_pred_(4, i) = psiDot + delta_t*nuPsiDotDot;
	}

	
	//predict state mean
	x_.setZero();
	for (int i = 0; i<2 * n_aug_ + 1; i++)
		x_ += weights_(i)*Xsig_pred_.col(i);


	//predict state covariance matrix
	P_.setZero();
	MatrixXd xCurr = MatrixXd(n_x_, 1);
	for (int i = 0; i<2 * n_aug_ + 1; i++)
	{
		xCurr = Xsig_pred_.col(i) - x_;
		//angle normalization
		while (xCurr(3)> M_PI) xCurr(3) -= 2.*M_PI;
		while (xCurr(3)<-M_PI) xCurr(3) += 2.*M_PI;

		P_ += weights_(i)*xCurr*xCurr.transpose();
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
	VectorXd z = meas_package.raw_measurements_;

	//dimensions of the measurement vector
	int n_z = 2;

	//create matrix for sigma points in measurement space
	MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

	//mean predicted measurement
	VectorXd z_pred = VectorXd(n_z);

	//measurement covariance matrix S
	MatrixXd S = MatrixXd(n_z, n_z);

	float px, py, v, psi;
	//transform sigma points into measurement space
	for (int i = 0; i<2 * n_aug_ + 1; i++)
	{
		px = Xsig_pred_(0, i);
		py = Xsig_pred_(1, i);
		v = Xsig_pred_(2, i);
		psi = Xsig_pred_(3, i);

		Zsig(0, i) = px;
		Zsig(1, i) = py;
	}

	//calculate mean predicted measurement
	z_pred.setZero();
	for (int i = 0; i<2 * n_aug_ + 1; i++)
		z_pred += Zsig.col(i)*weights_(i);

	//calculate innovation covariance matrix S
	S.setZero();
	MatrixXd xCurr = MatrixXd(n_x_, 1);
	for (int i = 0; i<2 * n_aug_ + 1; i++)
	{
		xCurr = Zsig.col(i) - z_pred;
		S += weights_(i)*xCurr*xCurr.transpose();
	}

	//measurement noise covariance matrix S
	MatrixXd R = MatrixXd(n_z, n_z);
	R.setZero();
	R(0, 0) = std_laspx_*std_laspx_;
	R(1, 1) = std_laspy_*std_laspy_;
	

	//add the noise
	S += R;
	
	//Now update the state vector and the state covariance matrix

	//create matrix for cross correlation Tc
	MatrixXd Tc = MatrixXd(n_x_, n_z);

	//calculate cross correlation matrix
	Tc.setZero();
	for (int i = 0; i<2 * n_aug_ + 1; i++)
	{
		//residual
		VectorXd z_diff = Zsig.col(i) - z_pred;
		
		// state difference
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		
		Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
	}

	//calculate Kalman gain K;
	MatrixXd K = MatrixXd(n_x_, n_z);
	K = Tc*S.inverse();

	//update state mean and covariance matrix
	//residual
	VectorXd z_diff = z - z_pred;
	double nis = z_diff.transpose()*S.inverse()*z_diff;
	cout << nis << endl;

	x_ = x_ + K*z_diff;
	P_ = P_ - K*S*K.transpose();
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
  
   VectorXd z = meas_package.raw_measurements_;

   //dimensions of the measurement vector
   int n_z = 3;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);

  float px, py, v, psi;
  //transform sigma points into measurement space
  for (int i = 0; i<2 * n_aug_ + 1; i++)
  {
	  px = Xsig_pred_(0, i);
	  py = Xsig_pred_(1, i);
	  v = Xsig_pred_(2, i);
	  psi = Xsig_pred_(3, i);

	  Zsig(0, i) = sqrt(px*px + py*py);
	  Zsig(1, i) = atan2(py, px);
	  if (Zsig(0, i) != 0)
		  Zsig(2, i) = (px*cos(psi)*v + py*sin(psi)*v) / Zsig(0, i);
	  else
		  Zsig(2, i) = 0;
  }

  //calculate mean predicted measurement
  z_pred.setZero();
  for (int i = 0; i<2 * n_aug_ + 1; i++)
	  z_pred += Zsig.col(i)*weights_(i);

  //calculate innovation covariance matrix S
  S.setZero();
  MatrixXd xCurr = MatrixXd(n_x_, 1);
  for (int i = 0; i<2 * n_aug_ + 1; i++)
  {
	  xCurr = Zsig.col(i) - z_pred;
	  S += weights_(i)*xCurr*xCurr.transpose();
  }

  //measurement noise covariance matrix S
  MatrixXd R = MatrixXd(n_z, n_z);
  R.setZero();
  R(0, 0) = std_radr_*std_radr_;
  R(1, 1) = std_radphi_*std_radphi_;
  R(2, 2) = std_radrd_*std_radrd_;

  //add the noise
  S += R;


  //Now update the state vector and the state covariance matrix

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  Tc.setZero();
  for (int i = 0; i<2 * n_aug_ + 1; i++)
  {
	  //residual
	  VectorXd z_diff = Zsig.col(i) - z_pred;
	  //angle normalization
	  while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
	  while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

	  // state difference
	  VectorXd x_diff = Xsig_pred_.col(i) - x_;
	  //angle normalization
	  while (x_diff(3)> M_PI) x_diff(3) -= 2.*M_PI;
	  while (x_diff(3)<-M_PI) x_diff(3) += 2.*M_PI;

	  Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //calculate Kalman gain K;
  MatrixXd K = MatrixXd(n_x_, n_z);
  K = Tc*S.inverse();

  //update state mean and covariance matrix
  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

  double nis = z_diff.transpose()*S.inverse()*z_diff;
  //cout << nis << endl;

  x_ = x_ + K*z_diff;
  P_ = P_ - K*S*K.transpose();
  //cout << P_ << endl;
}
