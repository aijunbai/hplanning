#ifndef STATISTIC_H
#define STATISTIC_H

#include <math.h>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <boost/shared_ptr.hpp>
#include <boost/program_options.hpp>
#include "distribution.h"
#include "utils.h"

template <class COUNT>
class VALUE {
 public:
  VALUE() {
    Count = 0;
    Total = 0;
  }

  void Set(double count, double value) {
    Count = count;
    Total = value * count;
  }

  void Add(double x) {
    Count += 1.0;
    Total += x;
  }

  void Add(double x, COUNT weight) {
    Count += weight;
    Total += x * weight;
  }

  double GetValue() const { return Count == 0 ? Total : Total / double(Count); }

  COUNT GetCount() const { return Count; }

 private:
  COUNT Count;
  double Total;
};

class STATISTIC {
 public:
  STATISTIC();

  ~STATISTIC() {}

  int GetCount() const;
  double GetValue() const;            // merge from VALUE
  void Set(int count, double value);  // merge from VALUE

  void Initialise();
  double GetTotal() const;
  double GetMean() const;
  double GetVariance() const;
  double GetStdDev() const;  //标准差
  double GetStdErr() const;  //标准误差（平均值的标准差）
  double GetMax() const;
  double GetMin() const;

  void SetMin(double min) { Min = min; }

  void SetMax(double max) { Max = max; }

  void AdjustRange(double min, double max) {
    Min = std::min(min, Min);
    Max = std::max(max, Max);
  }

  void Print(const std::string &name, std::ostream &ostr) const;
  void Add(double val);

  friend std::ostream &operator<<(std::ostream &os, const STATISTIC &o) {
    o.Print("", os);
    return os;
  }

 private:
  double Count;
  double Mean;      //平均值
  double Variance;  //方差
  double Min, Max;
};

inline STATISTIC::STATISTIC() { Initialise(); }

inline void STATISTIC::Set(int count, double value) {
  Initialise();

  Count = count;
  Mean = value;
}

inline void STATISTIC::Add(double val) {
  double meanOld = Mean;
  int countOld = Count;

  ++Count;
  assert(Count > 0);  // overflow
  Mean += (val - Mean) / Count;
  Variance = (countOld * (Variance + meanOld * meanOld) + val * val) / Count -
             Mean * Mean;

  if (Variance < 0.0) Variance = 0.0;
  if (val > Max) Max = val;
  if (val < Min) Min = val;
}

inline void STATISTIC::Initialise() {
  Count = 0;
  Mean = 0;
  Variance = 0;
  Min = +10e6;
  Max = -10e6;
}

inline int STATISTIC::GetCount() const { return Count; }

inline double STATISTIC::GetTotal() const { return Mean * Count; }

inline double STATISTIC::GetValue() const { return GetMean(); }

inline double STATISTIC::GetMean() const { return Mean; }

inline double STATISTIC::GetVariance() const { return Variance; }

inline double STATISTIC::GetStdDev() const { return sqrt(Variance); }

inline double STATISTIC::GetStdErr() const { return Count > 0? sqrt(Variance / Count): Infinity; }

inline double STATISTIC::GetMax() const { return Max; }

inline double STATISTIC::GetMin() const { return Min; }

inline void STATISTIC::Print(const std::string &name,
                             std::ostream &ostr) const {
  ostr << name << ": " << Mean << " (" << GetCount() << ") [" << Min << ", "
       << Max << "] +- " << GetStdErr() << ", sigma=" << GetStdDev()
       << std::endl;
}

class UniformGenerator {
 public:
  UniformGenerator(double low, double high) : mLow(low), mHigh(high) {}

  double operator()() { return SimpleRNG::ins().GetUniform(mLow, mHigh); }

  const double mLow;
  const double mHigh;
};

class NormalGammaGenerator {
 public:
  NormalGammaGenerator(double mu, double lambda, double alpha, double beta)
      : Mu(mu), Lambda(lambda), Alpha(alpha), Beta(beta) {}

  double operator()() {
    const double t = SimpleRNG::ins().GetGamma(Alpha, 1.0 / Beta);
    const double p = std::max(Lambda * t, 1.0e-6);
    const double m = SimpleRNG::ins().GetNormal(Mu, sqrt(1.0 / p));

    return m;
  }

 private:
  const double Mu;
  const double Lambda;
  const double Alpha;
  const double Beta;
};

class BetaInfo {
 public:
  BetaInfo() { Initialise(); }

  ~BetaInfo() {}

  void Initialise() {
    Alpha = ALPHA;
    Beta = BETA;
  }

  void Set(int count, double value) {
    Initialise();

    for (int i = 0; i < count; ++i) {
      Add(value);
    }
  }

  void Add(double value) {  // add a new sample
    double prob = (value - MIN) / (MAX - MIN);

    if (prob <= 0.0) {
      Beta += 1.0;
    } else if (prob >= 1.0) {
      Alpha += 1.0;
    } else {
      double trial = SimpleRNG::ins().GetUniform();

      if (trial < prob) {  // sucess
        Alpha += 1.0;
      } else {
        Beta += 1.0;
      }
    }
  }

  double GetExpectation() const { return ThompsonSampling(false); }

  double ThompsonSampling(bool sampling = true)
      const {  // Two Step: 采样一个模型参数，并计算出该模型参数对应的期望收益
    if (sampling) {
      double x = SimpleRNG::ins().GetGamma(Alpha);
      double y = SimpleRNG::ins().GetGamma(Beta);

      return x / (x + y) * (MAX - MIN) + MIN;
    } else {
      return Alpha / (Alpha + Beta) * (MAX - MIN) + MIN;
    }
  }

  void Print(const std::string &name, std::ostream &ostr) const {
    ostr << name << "{"
         << "a=" << Alpha << " b=" << Beta << " p=" << Alpha / (Alpha + Beta)
         << " m=" << GetExpectation() << "}";
  }

  static void setMinMax(double min, double max) {
    MIN = min;
    MAX = max;
  }

 private:
  static double MIN;
  static double MAX;
  static double ALPHA;
  static double BETA;

  double Alpha;
  double Beta;
};

class NormalGammaInfo {
 public:
  NormalGammaInfo() : Mu(0.0), Lambda(0.0), Alpha(ALPHA), Beta(BETA) {
    Initialise();
  }

  ~NormalGammaInfo() {}

  void Initialise() {
    Mu = 0.0;
    Lambda = 0.0;
    Alpha = ALPHA;
    Beta = BETA;
  }

  double GetValue() const { return Mu; }

  double GetCount() const { return Lambda; }

  double GetAlpha() const { return Alpha; }

  double GetBeta() const { return Beta; }

  double GetExpectation() const { return ThompsonSampling(false); }

  void Set(int count, double value) {
    Mu = value;
    Lambda = count;
    Alpha = ALPHA;
    Beta = BETA;
  }

  void Add(const double value) {  // add a new sample
    double n = 1;
    double m = value;
    double s = 0.0;

    double mu = (Lambda * Mu + n * m) / (Lambda + n);
    double lambda = Lambda + n;
    double alpha = Alpha + 0.5 * n;
    double beta = Beta + 0.5 * n * s +
                  0.5 * (Lambda * n / (Lambda + n)) * (m - Mu) * (m - Mu);

    Mu = mu;
    Lambda = lambda;
    Alpha = alpha;
    Beta = beta;
  }

  double ThompsonSampling(bool sampling = true) const {
    // Two Step: 采样一个模型参数，并计算出该模型参数对应的期望收益
    return sampling ? NormalGammaGenerator(Mu, Lambda, Alpha, Beta)() : Mu;
  }

  void Print(const std::string &name, std::ostream &ostr) const {
    ostr << name << ":"
         << " mu=" << Mu << " lambda=" << Lambda << " alpha=" << Alpha
         << " beta=" << Beta << " error=" << sqrt(Beta / (Lambda * (Alpha - 1)))
         << " sigma=" << sqrt(Beta / (Alpha - 1)) << std::endl;
  }

  static void SetALPHA(double alpha) { ALPHA = alpha; }

  static void SetBETA(double beta) { BETA = beta; }

 private:
  double Mu;
  double Lambda;
  double Alpha;
  double Beta;

  static double ALPHA;
  static double BETA;
};

template <class T, class H>
class DirichletInfo {
 public:
  const DirichletInfo<T, H> &operator=(const DirichletInfo<T, H> &o) {
    Alpha = o.Alpha;

    return *this;
  }

  const std::vector<std::pair<T, double>> &GetExpectation() const {
    return ThompsonSampling(false);
  }

  void Clear() { Alpha.clear(); }

  void Add(const T x) { Alpha[x] += 1.0; }

  void Set(const int count, const T x) {
    Clear();

    for (int i = 0; i < count; ++i) {
      Add(x);
    }
  }

  const std::vector<std::pair<T, double>> &ThompsonSampling(
      bool sampling = true)
      const {  // Two Step: 采样一个模型参数，并计算出该模型参数对应的期望收益
    outcomes_.clear();

    double sum = 0.0;
    for (typename H::iterator it = Alpha.begin(); it != Alpha.end(); ++it) {
      outcomes_.push_back(std::make_pair(it->first, 0));
      outcomes_.back().second =
          sampling ? SimpleRNG::ins().GetGamma(it->second) : it->second;
      sum += outcomes_.back().second;
    }

    for (typename std::vector<std::pair<T, double>>::iterator it =
             outcomes_.begin();
         it != outcomes_.end(); ++it) {
      it->second /= sum;
    }

    return outcomes_;
  }

  void Print(const std::string &name, std::ostream &ostr) const {
    const std::vector<std::pair<T, double>> &outcomes = GetExpectation();

    ostr << name << ":[";
    for (typename std::vector<std::pair<T, double>>::const_iterator it =
             outcomes.begin();
         it != outcomes.end(); ++it) {
      if (it != outcomes.begin()) {
        ostr << ", ";
      }

      ostr << "#" << it->first << "(" << it->second << ")";
    }
    ostr << "]";
  }

 private:
  mutable H Alpha;
  mutable std::vector<std::pair<T, double>> outcomes_;
};

template <typename T>
class DirichletInfo_POMCP
    : public DirichletInfo<T, std::unordered_map<T, double>> {};

class NormalGammaInfo_POMCP
    : public std::unordered_map<size_t, NormalGammaInfo> {};

#endif  // STATISTIC
