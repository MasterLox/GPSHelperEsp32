#ifndef GPSHelperPro_h
#define GPSHelperPro_h

#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <math.h>

class GPSHelperPro {
  public:
    GPSHelperPro(int gps_rx_pin, int gps_tx_pin);
    void begin();
    bool fetch();
    float getLongitude();
    float getLatitude();
    float getAltitude();
    int getSatellites();
    float getAngle(float targetLat, float targetLong);
    unsigned long getTimeMS();  // New function declaration
    String getTime();           // Function to return formatted GPS time (HH:MM:SS)
    float getSpeed();           // Function to calculate GPS speed (km/h)

  private:
    HardwareSerial gpsSerial;
    TinyGPSPlus gps;
    float filteredLongitude = 0.0;
    float filteredLatitude = 0.0;
    bool isFirstReading = true;

    void getFilteredGPS(float& longitude, float& latitude);
    float calculateAngle(float targetLat, float targetLong, float currentLat, float currentLong);
    float calculateDistance(double lat1, double lon1, double lat2, double lon2); // Function to calculate distance between two GPS points

    float longitude;
    float latitude;
    float altitude;
    int satellites;
};

#endif
