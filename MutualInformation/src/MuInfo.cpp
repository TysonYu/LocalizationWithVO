//
// Created by hyye on 2/22/19.
//

#include "MuInfo.h"

void MuInfo::CalHistogram() {

  hist.Reset();

  for (int i = 0; i < depth_images.size(); ++i) {
    cv::Mat depth = depth_images[i];
    cv::Mat intenisty = gray_images[i];

    for (int v = 0; v < depth.rows; ++v) {
      for (int u = 0; u < depth.cols; ++u) {
        // WARNING: not real depth
        int refc = depth.at<uchar>(v, u) / fraction;
        int gray = intenisty.at<uchar>(v, u) / fraction;
        if (refc > 0) { // threshold for not to small
          hist.grayHist.at<float>(gray) += 1;
          hist.refcHist.at<float>(refc) += 1;
          hist.jointHist.at<float>(gray, refc) += 1;
          ++hist.count;
          hist.gray_sum += gray;
          hist.refc_sum += refc;
        }
      }
    }
  }
}

void MuInfo::CalProbability() {
  prob.Reset();

  CalHistogram();

  float mu_gray = hist.gray_sum / hist.count;
  float mu_refc = hist.refc_sum / hist.count;

  // variance
  double sigma_gray = 0;
  double sigma_refc = 0;
  // covariance
  double sigma_gr = 0;

  for (int i = 0; i < this->bins; i++) {
    for (int j = 0; j < this->bins; j++) {
      //Cross Correlation term;
      sigma_gr = sigma_gr + hist.jointHist.at<float>(i, j) * (i - mu_gray) * (j - mu_refc);
      //Normalize the Histograms so that the value is between (0,1)
      prob.jointProb.at<float>(i, j) = hist.jointHist.at<float>(i, j) / (hist.count);
    }

    //calculate sample covariance
    sigma_gray = sigma_gray + (hist.grayHist.at<float>(i) * (i - mu_gray) * (i - mu_gray));
    sigma_refc = sigma_refc + (hist.refcHist.at<float>(i) * (i - mu_refc) * (i - mu_refc));

    prob.grayProb.at<float>(i) = hist.grayHist.at<float>(i) / hist.count;
    prob.refcProb.at<float>(i) = hist.refcHist.at<float>(i) / hist.count;
  }

  sigma_gray = sigma_gray / hist.count;
  sigma_refc = sigma_refc / hist.count;
  sigma_gr = sigma_gr / hist.count;
  corr_coeff = ((sigma_gr) / (sigma_gray * sigma_refc));
  corr_coeff = sqrt(corr_coeff * corr_coeff);

  //Compute the optimal bandwidth (Silverman's rule of thumb)
  sigma_gray = 1.06 * sqrt(sigma_gray) / pow(hist.count, 0.2);
  sigma_refc = 1.06 * sqrt(sigma_refc) / pow(hist.count, 0.2);

  // WARNING: no KDE
  cv::GaussianBlur(prob.grayProb, prob.grayProb, cv::Size(0, 0), sigma_gray);
  cv::GaussianBlur(prob.refcProb, prob.refcProb, cv::Size(0, 0), sigma_refc);
  // refc is on x axis, horizontally in image; gray is on y axis, vertically in image
  cv::GaussianBlur(prob.jointProb, prob.jointProb, cv::Size(0, 0), sigma_refc, sigma_gray);
  prob.count = hist.count;

}

float MuInfo::GetCost() {
  CalProbability();
  cv::Mat jointLog = cv::Mat::zeros(bins, bins, CV_32FC1);
  cv::Mat grayLog = cv::Mat::zeros(1, bins, CV_32FC1);
  cv::Mat refcLog = cv::Mat::zeros(1, bins, CV_32FC1);
  cv::log(prob.jointProb, jointLog);
  cv::log(prob.grayProb, grayLog);
  cv::log(prob.refcProb, refcLog);
  cv::Mat jointEntropyMat, grayEntropyMat, refcEntropyMat;
  //jointEntropyMat = jointJSEstimate*jointJSLog;
  cv::multiply(prob.jointProb, jointLog, jointEntropyMat);
  //margEntropyMat1 = margJSEstimate1*margJSLog1;
  cv::multiply(prob.grayProb, grayLog, grayEntropyMat);
  //margEntropyMat2 = margJSEstimate2*margJSLog2;
  cv::multiply(prob.refcProb, refcLog, refcEntropyMat);
  float Hx = cv::norm(grayEntropyMat, cv::NORM_L1);
  float Hy = cv::norm(refcEntropyMat, cv::NORM_L1);
  float Hxy = cv::norm(jointEntropyMat, cv::NORM_L1);
  float cost = (Hx + Hy) - Hxy;

  return cost;
}