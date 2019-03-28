//
// Created by hyye on 2/22/19.
//

#ifndef LI_CAM_MUINFO_H_
#define LI_CAM_MUINFO_H_

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <Eigen/StdVector>

#define MAX_BINS 256

struct Probability {
  Probability(int n) {
    bins = n;
    Reset();
  };

  void Reset() {
    jointProb = cv::Mat::zeros(bins, bins, CV_32FC1);
    refcProb = cv::Mat::zeros(1, bins, CV_32FC1);
    grayProb = cv::Mat::zeros(1, bins, CV_32FC1);
    count = 0;
  }
  //joint Probability
  cv::Mat jointProb;
  //marginal probability reflectivity
  cv::Mat refcProb;
  //marginal probability grayscale
  cv::Mat grayProb;
  int count;
  int bins;
};

struct Histograms {
  Histograms(int n) {
    bins = n;
    Reset();
  };
  void Reset() {
    jointHist = cv::Mat::zeros(bins, bins, CV_32FC1);
    refcHist = cv::Mat::zeros(1, bins, CV_32FC1);
    grayHist = cv::Mat::zeros(1, bins, CV_32FC1);
    count = 0;
    gray_sum = 0;
    refc_sum = 0;
  }

  int bins;
  //joint Histograms
  cv::Mat jointHist;
  cv::Mat refcHist;
  cv::Mat grayHist;
  int count;
  int gray_sum;
  int refc_sum;
};

struct MuInfo {
  MuInfo(int num_fraction = 1) : fraction{num_fraction}, bins{MAX_BINS / fraction}, hist{bins}, prob{bins} {}

  void CalHistogram();
  void CalProbability();
  float GetCost();

  int fraction;
  int bins;
  Histograms hist;
  Probability prob;

  double corr_coeff;

  std::vector<cv::Mat> gray_images;
  std::vector<cv::Mat> depth_images;
};

#endif //LI_CAM_MUINFO_H_
