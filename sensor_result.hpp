class SensorResult {
  public:
    SensorResult();
    SensorResult(int sensor_id, int value);
    int getSensorId();
    int getValue();

  private:
    int sensor_id;
    int value;
};