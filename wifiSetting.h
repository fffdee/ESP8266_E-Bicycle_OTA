#ifndef WIFISETTING_H_
#define WIFISETTING_H_

#ifndef APSSID
#define APSSID "ESP8266AP"
#define APPSK  "88888888"
#endif
/* Set these to your desired credentials. */
const char* host = "esp8266-webupdate";
const char *ssid = APSSID;
const char *password = APPSK;

const char* serverIndex = "<form method='POST' action='/update' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>";

#endif