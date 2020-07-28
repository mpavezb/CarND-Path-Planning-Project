#ifndef PREDICTION_H_
#define PREDICTION_H_

#include "data_types.h"

namespace udacity {
class Prediction {
 public:
  void setTelemetry(const TelemetryPacket& telemetry) {
    telemetry_ = telemetry;
  }

  void step() {
    // Do something!
    beliefs.sensor_fusion = telemetry_.sensor_fusion;
  }

  PredictionData getPredictions() { return beliefs; }

 private:
  TelemetryPacket telemetry_;
  PredictionData beliefs;
};

}  // namespace udacity

#endif  // PREDICTION_H_
