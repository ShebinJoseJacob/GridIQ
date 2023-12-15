#include <EOWIAM_inferencing.h>
#include <ACS712.h>
#include <ZMPT101B.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>

#define SENSITIVITY 500.0f

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

ZMPT101B voltageSensor(4, 50.0);
ACS712  ACS(1,3.3, 4095, 185);

String DEVICE_ID = "LIGHT1";
int actuatorPIN = 5; //Relay PIN

String DEVICE_NAME = "Light";

WiFiClient wifi;
PubSubClient mqttClient(wifi);


long Current_Time, Loop_Time;
float mA, volt, average;
float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

const char* mqttServer = "test.mosquitto.org";
const int mqttPort = 1883;
const char* mqttClientId = "myClientIDs";

const char* ssid = "<WiFi SSID>";
const char* password = "<WiFi PASSWORD>";

//Twilio Creds
const char* account_sid = "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";
const char* auth_token = "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";
const char* from_number = "XXXXXXXXXXXXX";
const char* to_number = "XXXXXXXXXXXXX";

unsigned long lastTime = 0; 
unsigned long currentTime = 0;
bool deviceOn = false; 
float totalEnergyUsed = 0.0; 
float lastPower = 0.0;

void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
}

void connectToMQTTServer() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT server...");
    if (mqttClient.connect(mqttClientId)) {
      Serial.println("Connected");
      mqttClient.subscribe("UID123/Command");
    } else {
      Serial.print("Failed, retrying in 5 seconds...");
      delay(5000);
    }
  }
}

void reconnectToMQTTServer() {
  if (!mqttClient.connected()) {
    Serial.println("MQTT connection lost. Reconnecting...");
    connectToMQTTServer();
  }
}

void handleCondition(String receivedData){
  if (receivedData == (DEVICE_ID + ":1")){
    Serial.println(DEVICE_ID + " ON");
    digitalWrite(actuatorPIN, HIGH);
  }
  else if(receivedData==(DEVICE_ID + ":0")){
    Serial.println(DEVICE_ID + " OFF");
    digitalWrite(actuatorPIN, LOW);
  }
  else{
    Serial.println("Invalid Condition");
  }
}

void subscribeReceive(char* topic, byte* payload, unsigned int length) {
  String mqttMessage = String((char*)payload, length);
  std::string mqttMessageStr = mqttMessage.c_str();

  Serial.print("Received Message: ");
  Serial.println(mqttMessage);
  handleCondition(mqttMessage);
  
}

float calculatePower(){
  return ((mA/1000.0)*volt);
}

void updateFeatures() {
    for (int i = 0; i < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; i += 2) {
     Current_Time = millis();
     if(Current_Time >= (Loop_Time + 250))
     {
        Loop_Time = Current_Time;
        ACS.suppressNoise(true);
        average = 0;
        for (int i = 0; i < 10; i++)
        {
          average += ACS.mA_AC();
        }
        mA = average/10.0;
        volt = voltageSensor.getRmsVoltage();
     }  
     features[i + 0] = mA;
     features[i + 1] = volt;
     delay(EI_CLASSIFIER_INTERVAL_MS);
    }
   String topic = "UID123/Sensors";
   String Current = "Current:" + String(mA);
   String Voltage = "Voltage:" + String(volt);
   Serial.println(Current);
   Serial.println(Voltage);
   mqttClient.publish(topic.c_str(), Current.c_str());
   mqttClient.publish(topic.c_str(), Voltage.c_str());
}


int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, features + offset, length * sizeof(float));
    return 0;
}

void sendTwilioMessage() {
  HTTPClient http;

  String url = "https://api.twilio.com/2010-04-01/Accounts/";
  url += account_sid;
  url += "/Messages.json";

  String message = DEVICE_NAME + " is malfunctioning!";

  String postData = "To=";
  postData += to_number;
  postData += "&From=";
  postData += from_number;
  postData += "&Body=";
  postData += message;

  http.begin(url);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  http.setAuthorization(account_sid, auth_token);

  int httpResponseCode = http.POST(postData);

  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println(httpResponseCode);
    Serial.println(response);
  } else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }

  http.end();
}

void print_inference_result(ei_impulse_result_t result);

void setup()
{ 
  Serial.begin(115200);
  pinMode(actuatorPIN, OUTPUT);
  setup_wifi();
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(subscribeReceive);
  connectToMQTTServer();

  timeClient.begin();
  timeClient.setTimeOffset(19800); // UTC offset for IST (5 * 3600 + 30* 60)
  
  voltageSensor.setSensitivity(SENSITIVITY);
  ACS.autoMidPoint();
  Serial.print("MidPoint: ");
  Serial.println(ACS.getMidPoint());
  Serial.print("Noise mV: ");
  Serial.println(ACS.getNoisemV());
}

void loop()
{   
    timeClient.update();
    mqttClient.loop();
    Serial.println(timeClient.getFormattedTime());
    
    if (timeClient.getFormattedTime() == "18:00:00"){
      digitalWrite(actuatorPIN, LOW);
    }
    currentTime = millis();

    if (sizeof(features) / sizeof(float) != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        ei_printf("The size of your 'features' array is not correct. Expected %lu items, but had %lu\n",
            EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, sizeof(features) / sizeof(float));
        delay(1000);
        return;
    }

    ei_impulse_result_t result = { 0 };

    updateFeatures();
 
    signal_t features_signal;
    features_signal.total_length = sizeof(features) / sizeof(features[0]);
    features_signal.get_data = &raw_feature_get_data;

    EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false /* debug */);
    if (res != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", res);
        return;
    }
    
    ei_printf("run_classifier returned: %d\r\n", res);
    print_inference_result(result);
    
    delay(1000);
}

void print_inference_result(ei_impulse_result_t result) {
  ei_printf("Timing: DSP %d ms, inference %d ms, anomaly %d ms\r\n",
          result.timing.dsp,
          result.timing.classification,
          result.timing.anomaly);

  ei_printf("Predictions:\r\n");
  for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
      ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
      ei_printf("%.5f\r\n", result.classification[i].value);

      if (result.classification[i].value > 0.7 and ei_classifier_inferencing_categories[i]=="Light_On" and deviceOn == false){
        deviceOn = true;
        lastTime = currentTime;
        lastPower = calculatePower(); 
        Serial.println("Device turned on. Power Usage");
        Serial.println(totalEnergyUsed);
      }
      else if (result.classification[i].value > 0.7 and ei_classifier_inferencing_categories[i]=="Off" and deviceOn== true){
        deviceOn = false;

        Serial.print("Device turned off. Total Energy Usage: ");
        Serial.print(totalEnergyUsed);
        Serial.println(" Wh");

        String topic = "UID123/Sensors";
        String energy = "Energy:"+String(totalEnergyUsed);
        mqttClient.publish(topic.c_str(), energy.c_str());

      }
  }

  if (deviceOn && currentTime - lastTime >= 1000) {
    lastPower = calculatePower(); 
    Serial.print("Power Usage: ");
    Serial.println(lastPower);
    
    float timeInSeconds = (currentTime - lastTime) / 1000.0;
    float energyUsed = (lastPower * timeInSeconds) / 3600.0;
    totalEnergyUsed += energyUsed;
    
    Serial.print("Energy used: ");
    Serial.print(totalEnergyUsed);
    Serial.println(" Wh");

    String topic = "UID123/Sensors";
    String energy = "Energy:"+String(totalEnergyUsed);
    mqttClient.publish(topic.c_str(), energy.c_str());
    lastTime = currentTime;
  }

  #if EI_CLASSIFIER_HAS_ANOMALY == 1
      ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
      if (result.anomaly > 30){
        String topic = "UID123/Sensors";
        String anomaly = "Anomaly:1";
        mqttClient.publish(topic.c_str(), anomaly.c_str());
        Serial.println(anomaly);
        sendTwilioMessage();
        
      }
      else{
        String topic = "UID123/Sensors";
        String anomaly = "Anomaly:0";
        mqttClient.publish(topic.c_str(), anomaly.c_str());
        Serial.println(anomaly);
      }
  #endif

}
