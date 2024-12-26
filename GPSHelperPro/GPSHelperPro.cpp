#include "GPSHelperPro.h"

GPSHelperPro::GPSHelperPro(int gps_rx_pin, int gps_tx_pin) : gpsSerial(1) {
  gpsSerial.begin(115200, SERIAL_8N1, gps_rx_pin, gps_tx_pin);
}

void GPSHelperPro::begin() {
  Serial.begin(9600);
  while (!Serial) {
    delay(10);
  }
}

bool GPSHelperPro::fetch() {
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    gps.encode(c);
  }

  if (gps.location.isUpdated() && gps.altitude.isUpdated() && gps.satellites.isUpdated()) {
    longitude = gps.location.lng();
    latitude = gps.location.lat();
    altitude = gps.altitude.meters();
    satellites = gps.satellites.value();
    return true;
  }

  return false;
}

float GPSHelperPro::getLongitude() {
  return longitude;
}

float GPSHelperPro::getLatitude() {
  return latitude;
}

float GPSHelperPro::getAltitude() {
  return altitude;
}

int GPSHelperPro::getSatellites() {
  return satellites;
}

float GPSHelperPro::getAngle(float tarLat, float tarLng) {
  getFilteredGPS(longitude, latitude);
  return calculateAngle(tarLat, tarLng, latitude, longitude);
}

void GPSHelperPro::getFilteredGPS(float& longitude, float& latitude) {
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    gps.encode(c);
  }

  if (gps.location.isUpdated() && gps.hdop.isValid() && gps.hdop.hdop() < 3) {
    if (isFirstReading) {
      filteredLongitude = gps.location.lng();
      filteredLatitude = gps.location.lat();
      isFirstReading = false;
    }
    
    filteredLongitude = 0.2 * gps.location.lng() + 0.8 * filteredLongitude;
    filteredLatitude = 0.2 * gps.location.lat() + 0.8 * filteredLatitude;
  }
  
  longitude = filteredLongitude;
  latitude = filteredLatitude;
}

float GPSHelperPro::calculateAngle(float targetLat, float targetLong, float currentLat, float currentLong) {
  float lat1Rad = radians(currentLat);
  float long1Rad = radians(currentLong);
  float lat2Rad = radians(targetLat);
  float long2Rad = radians(targetLong);
  float dLong = long2Rad - long1Rad;
  float angle = atan2(sin(dLong) * cos(lat2Rad), 
                      cos(lat1Rad) * sin(lat2Rad) - sin(lat1Rad) * cos(lat2Rad) * cos(dLong));
  angle = degrees(angle);
  angle = fmod(angle + 360, 360);
  return angle;
}

// Get the current time in HH:MM:SS format
String GPSHelperPro::getTime() {
  if (gps.time.isUpdated()) {
    int hour = gps.time.hour();
    int minute = gps.time.minute();
    int second = gps.time.second();
    return String(hour) + ":" + String(minute) + ":" + String(second);
  }
  return "00:00:00";  // Return a default value if time is not updated
}

// Method to calculate speed in km/h
float GPSHelperPro::getSpeed() {
  static double lastLat = 0.0, lastLng = 0.0;
  static unsigned long lastTime = 0;
  
  unsigned long currentTime = millis();
  
  // Only calculate speed if we have previous GPS data
  if (lastLat != 0.0 && lastLng != 0.0) {
    float distance = calculateDistance(lastLat, lastLng, latitude, longitude);
    float timeElapsed = (currentTime - lastTime) / 1000.0;  // Time elapsed in seconds
    
    // Calculate speed (distance/time) in km/h
    if (timeElapsed > 0) {
      float speed = (distance / timeElapsed) * 3.6;  // Convert to km/h
      lastTime = currentTime;
      lastLat = latitude;
      lastLng = longitude;
      return speed;
    }
  }
  
  // Update last position and time if it's the first time
  lastLat = latitude;
  lastLng = longitude;
  lastTime = currentTime;
  return 0.0; // Return 0 if no speed can be calculated yet
}

// Function to calculate the distance between two GPS coordinates (Haversine formula)
float GPSHelperPro::calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat / 2) * sin(dLat / 2) + cos(radians(lat1)) * cos(radians(lat2)) * sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float R = 6371; // Radius of Earth in kilometers
  return R * c; // Distance in km
}
