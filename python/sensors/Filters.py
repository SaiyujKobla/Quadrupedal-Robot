from __future__ import annotations


from dataclasses import dataclass
from typing import Optional


import numpy as np




@dataclass(frozen=True)
class Kalman1DConfig:
   process_acceleration_std: float = 1.5
   measurement_std: float = 0.03
   initial_position_std: float = 0.20
   initial_velocity_std: float = 1.00
   innovation_gate_sigma: float = 4.0
   maximum_prediction_step_s: float = 0.50




@dataclass(frozen=True)
class KalmanEstimate:
   value: Optional[float]
   velocity: Optional[float]
   accepted_measurement: bool
   predicted_only: bool
   innovation: Optional[float]




class ConstantVelocityKalman1D:
   def __init__(self, config: Optional[Kalman1DConfig] = None) -> None:
       self.config = config or Kalman1DConfig()
       self._state = np.zeros((2, 1), dtype=float)
       self._covariance = np.eye(2, dtype=float)
       self._initialized = False
       self._last_timestamp: Optional[float] = None


   @property
   def initialized(self) -> bool:
       return self._initialized


   def reset(self) -> None:
       self._state.fill(0.0)
       self._covariance = np.eye(2, dtype=float)
       self._initialized = False
       self._last_timestamp = None


   def initialize(self, measurement: float, timestamp: float) -> KalmanEstimate:
       self._state = np.array([[float(measurement)], [0.0]], dtype=float)
       self._covariance = np.diag(
           [
               self.config.initial_position_std**2,
               self.config.initial_velocity_std**2,
           ]
       )
       self._initialized = True
       self._last_timestamp = float(timestamp)
       return self.estimate(
           accepted_measurement=True,
           predicted_only=False,
           innovation=0.0,
       )


   def _predict_to(self, timestamp: float) -> None:
       if not self._initialized:
           return


       if self._last_timestamp is None:
           self._last_timestamp = float(timestamp)
           return


       dt = max(0.0, float(timestamp) - self._last_timestamp)
       dt = min(dt, self.config.maximum_prediction_step_s)
       self._last_timestamp = float(timestamp)


       if dt <= 0.0:
           return


       transition = np.array([[1.0, dt], [0.0, 1.0]], dtype=float)


       acceleration_variance = self.config.process_acceleration_std**2
       process_noise = acceleration_variance * np.array(
           [
               [0.25 * dt**4, 0.5 * dt**3],
               [0.5 * dt**3, dt**2],
           ],
           dtype=float,
       )


       self._state = transition @ self._state
       self._covariance = (
           transition @ self._covariance @ transition.T + process_noise
       )


   def step(
       self,
       measurement: Optional[float],
       timestamp: float,
       measurement_valid: bool = True,
   ) -> KalmanEstimate:
       usable_measurement = (
           measurement_valid
           and measurement is not None
           and np.isfinite(float(measurement))
       )


       if not self._initialized:
           if usable_measurement:
               return self.initialize(float(measurement), timestamp)
           return KalmanEstimate(None, None, False, True, None)


       self._predict_to(timestamp)


       if not usable_measurement:
           return self.estimate(
               accepted_measurement=False,
               predicted_only=True,
               innovation=None,
           )


       measurement_value = float(measurement)
       observation = np.array([[1.0, 0.0]], dtype=float)
       measurement_variance = self.config.measurement_std**2


       predicted_measurement = float((observation @ self._state)[0, 0])
       innovation = measurement_value - predicted_measurement
       innovation_variance = float(
           (observation @ self._covariance @ observation.T)[0, 0]
           + measurement_variance
       )


       gate = self.config.innovation_gate_sigma * np.sqrt(
           max(innovation_variance, 1e-12)
       )


       if abs(innovation) > gate:
           return self.estimate(
               accepted_measurement=False,
               predicted_only=True,
               innovation=innovation,
           )


       kalman_gain = self._covariance @ observation.T / innovation_variance
       self._state = self._state + kalman_gain * innovation


       identity = np.eye(2, dtype=float)
       self._covariance = (
           identity - kalman_gain @ observation
       ) @ self._covariance


       return self.estimate(
           accepted_measurement=True,
           predicted_only=False,
           innovation=innovation,
       )


   def estimate(
       self,
       accepted_measurement: bool = False,
       predicted_only: bool = True,
       innovation: Optional[float] = None,
   ) -> KalmanEstimate:
       if not self._initialized:
           return KalmanEstimate(None, None, False, True, innovation)


       return KalmanEstimate(
           value=float(self._state[0, 0]),
           velocity=float(self._state[1, 0]),
           accepted_measurement=accepted_measurement,
           predicted_only=predicted_only,
           innovation=innovation,
       )
