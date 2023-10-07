// Configuration
#define SEARCH_STRONG_AP

#define WIFI_HOSTNAME   "Radio"
#define WIFI_SSID       "---- Fill in my WiFi ---"
#define WIFI_PASSPHRASE "XXXXXXXXXXXXXXXXXXXXXXXX"

#define STREAM_URL      "http://mpd:8000/"
#define MPD_HOSTNAME    "mpd"
#define MPD_PORT        6600

#define longKeypress 8

#define luxThreshold 5
#define darkDuration 100
#define lightDuration 10
#define sleepDuration 100000

extern "C"
{
#include <soc/adc_periph.h>
#include <soc/rtc_wdt.h>
}

#include <Arduino.h>
#include <Audio.h>
#include <WiFi.h>
#ifdef SEARCH_STRONG_AP
#include <WiFiMulti.h>
#endif
#include <Wire.h>
#include <SPIFFS.h>
#include <NeoPixelBus.h>
#include <Seeed_TMG3993.h>

#define I2S_DOUT      26
#define I2S_BCLK      5
#define I2S_LRC       25
#define I2S_DIN       35
#define I2SN (i2s_port_t)0
#define I2CN (i2c_port_t)0
#define SDA 18
#define SCL 23

//Buttons
#define MU GPIO_NUM_12        // Play
#define VM GPIO_NUM_32        // volume -
#define VP GPIO_NUM_19        // volume +
#define STOP GPIO_NUM_12

//Amp power enable
#define PA GPIO_NUM_21

#define maxVol 33
#define maxAudio 22

//////////////////////////////
// NeoPixel led control
/////////////////////////////
#define PixelCount 1
#define PixelPin 22
RgbColor RED(255, 0, 0);
RgbColor GREEN(0, 255, 0);
RgbColor BLUE(0, 0, 255);
RgbColor YELLOW(255, 128, 0);
RgbColor WHITE(255, 255, 255);
RgbColor BLACK(0, 0, 0);

RgbColor REDL(64, 0, 0);
RgbColor GREENL(0, 64, 0);
RgbColor BLUEL(0, 0, 64);
RgbColor WHITEL(64, 64, 64);
RgbColor BLACKL(0, 0, 0);
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);

Audio audio;
TMG3993 tmg3993;
#ifdef SEARCH_STRONG_AP
WiFiMulti wifiMulti;
#endif

TaskHandle_t radioTask, batteryTask, keyboardTask;

int buttonPlus = -1,buttonMinus = -1,buttonPlay = -1;
int vol= maxVol / 2;
bool mute = false;

bool connected = false, started = false, hasTMG = false;
int darkCount = 0;

#define ES8388_ADDR 0x10
void ES8388_Write_Reg(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(ES8388_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

void ES8388vol_Set(uint8_t volx)
{
#define lowVol 8
  if (volx > maxVol)
    volx = maxVol;
  if (volx == 0)
    ES8388_Write_Reg(25, 0x04);
  else
    ES8388_Write_Reg(25, 0x00);

  if(volx > lowVol)
  {
    audio.setVolume(maxAudio);
    ES8388_Write_Reg(46, volx);
    ES8388_Write_Reg(47, volx);
    ES8388_Write_Reg(26, 0x00);
    ES8388_Write_Reg(27, 0x00); 
  }
  else
  {
    audio.setVolume(maxAudio*volx/lowVol);
    ES8388_Write_Reg(46, lowVol);
    ES8388_Write_Reg(47, lowVol);
    ES8388_Write_Reg(26, 0x00);
    ES8388_Write_Reg(27, 0x00);  
  }
}

void ES8388_Init(void)
{
  // provides MCLK
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
  WRITE_PERI_REG(PIN_CTRL, READ_PERI_REG(PIN_CTRL)& 0xFFFFFFF0);
  
  // reset
  ES8388_Write_Reg(0, 0x80);
  ES8388_Write_Reg(0, 0x00);
  // mute
  ES8388_Write_Reg(25, 0x04);
  ES8388_Write_Reg(1, 0x50);
  //powerup
  ES8388_Write_Reg(2, 0x00);
  // slave mode
  ES8388_Write_Reg(8, 0x00);
  // DAC powerdown
  ES8388_Write_Reg(4, 0xC0);
  // vmidsel/500k ADC/DAC idem
  ES8388_Write_Reg(0, 0x12);
  
  ES8388_Write_Reg(1, 0x00);
  // i2s 16 bits
  ES8388_Write_Reg(23, 0x18);
  // sample freq 256
  ES8388_Write_Reg(24, 0x02);
  // LIN2/RIN2 for mixer
  ES8388_Write_Reg(38, 0x09);
  // left DAC to left mixer
  ES8388_Write_Reg(39, 0x90);
  // right DAC to right mixer
  ES8388_Write_Reg(42, 0x90);
  // DACLRC ADCLRC idem
  ES8388_Write_Reg(43, 0x80);
  ES8388_Write_Reg(45, 0x00);
  // DAC volume max
  ES8388_Write_Reg(27, 0x00);
  ES8388_Write_Reg(26, 0x00);
  
  ES8388_Write_Reg(2 , 0xF0);
  ES8388_Write_Reg(2 , 0x00);
  ES8388_Write_Reg(29, 0x1C);
  // DAC power-up LOUT1/ROUT1 enabled
  ES8388_Write_Reg(4, 0x30);
  // unmute
  ES8388_Write_Reg(25, 0x00);
  // amp validation
  gpio_set_level(PA, 1);
  ES8388_Write_Reg(46, 33);
  ES8388_Write_Reg(47, 33); 
}

static void processButtons(void* pdata)
{
  static int durationPlus=0, durationMinus=0, durationPlay=0;
  static bool pressedPlus=false, pressedMinus=false, pressedPlay=false;

  while(true)
  {
    if(gpio_get_level(VP) == 0) // Pressed
    {
      durationPlus++;
      pressedPlus = true;
    }
    if(gpio_get_level(VP) == 1 && pressedPlus == 1)
    {
      buttonPlus = durationPlus;
      pressedPlus = false;
    }
    if(gpio_get_level(VP) == 1 && buttonPlus == -1)
    {
      // Button was handled.
      durationPlus = 0;
      pressedPlus = false;
    }

    if(gpio_get_level(VM) == 0) // Pressed
    {
      durationMinus++;
      pressedMinus = true;
    }
    if(gpio_get_level(VM) == 1 && pressedMinus == 1)
    {
      buttonMinus = durationMinus;
      pressedMinus = false;
    }
    if(gpio_get_level(VM) == 1 && buttonMinus == -1)
    {
      // Button was handled.
      durationMinus = 0;
      pressedMinus = false;
    }
   
    if(gpio_get_level(MU) == 0) // Pressed
    {
      durationPlay++;
      pressedPlay = true;
    }
    if(gpio_get_level(MU) == 1 && pressedPlay == 1)
    {
      buttonPlay = durationPlay;
      pressedPlay = false;
    }
    if(gpio_get_level(MU) == 1 && buttonPlay == -1)
    {
      // Button was handled.
      durationPlay = 0;
      pressedPlay = false;
    }

    delay(100);
  }
}

#define NGREEN 2300
#define NYELLOW 1800
static void showBattery(void* pdata)
{
  while(true)
  {
    int val = adc1_get_raw(ADC1_GPIO33_CHANNEL);

    if(val < NYELLOW)
       strip.SetPixelColor(0, RED);
    else if(val > NGREEN)
      strip.SetPixelColor(0, GREEN);
    else
      strip.SetPixelColor(0, YELLOW);
    strip.Show();   
    delay(10000);
  }
}

static void playRadio(void* data)
{
  bool startup = true;
  
  while (true)
  {
    while (started == false)
    {
       delay(100);
       startup = true;
    }
  
    if (connected == false)
    {
      if (startup)
      {
        i2s_stop(I2SN);
        i2s_zero_dma_buffer(I2SN);
        delay(500);
        i2s_start(I2SN);     
        audio.stopSong();
        
        startup = false;
      }
      
      audio.connecttohost(STREAM_URL);
    }

    audio.loop();
  }
}

void setupSPIFFS()
{
  if(!SPIFFS.begin(true))
  {
    Serial.println("Error initializing SPIFFS.");
    return;
  }
  
  File root = SPIFFS.open("/");
  File file = root.openNextFile();
  while(file)
  {
    Serial.print("FILE: ");
    Serial.println(file.name());
    file = root.openNextFile();
  }
  
  printf("====> %d\n",(int)SPIFFS.totalBytes());
  printf("====> %d\n",(int)SPIFFS.usedBytes());   
    
  printf(" SPIFFS used bytes  ====> %d of %d\n",(int)SPIFFS.usedBytes(), (int)SPIFFS.totalBytes());      
}

void setupLED()
{
  strip.Begin();
}

void setupBattery()
{
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_GPIO33_CHANNEL, ADC_ATTEN_DB_11);
}

void setupI2C()
{
  Wire.begin(SDA, SCL, 10000);
}

void setupGPIO()
{
  gpio_reset_pin(MU);
  gpio_reset_pin(VP);
  gpio_reset_pin(VM);

  gpio_set_direction(MU, GPIO_MODE_INPUT);  
  gpio_set_direction(VP, GPIO_MODE_INPUT);  
  gpio_set_direction(VM, GPIO_MODE_INPUT);  

  gpio_set_pull_mode(MU, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode(VP, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode(VM, GPIO_PULLUP_ONLY);

  gpio_reset_pin(PA);
  gpio_set_direction(PA, GPIO_MODE_OUTPUT);   
  gpio_set_level(PA, 1); 
}

void initVolume()
{
  char b[4] = "";
  File ln = SPIFFS.open("/volume", "r");
  ln.read((uint8_t*)b, 2);
  if (b[0])
  {
    b[2] = 0;
    vol = atoi(b);
  }
  ln.close();

  printf("Volume: %d (max: %d)\n", vol, maxVol);
}

void initAudio()
{
   audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
   ES8388_Init();
   ES8388vol_Set(vol);  
}

void initWiFi()
{
  WiFi.setHostname(WIFI_HOSTNAME);
#ifdef SEARCH_STRONG_AP
  wifiMulti.addAP(WIFI_SSID, WIFI_PASSPHRASE);
  wifiMulti.run();
#else
  WiFi.begin(WIFI_SSID, WIFI_PASSPHRASE);
#endif
}

void initTMG()
{
  if (!tmg3993.initialize())
    return;
  tmg3993.setADCIntegrationTime(0xdb); // the integration time: 103ms
  tmg3993.enableEngines(ENABLE_PON | ENABLE_AEN | ENABLE_AIEN);
  
  hasTMG = true;
}

void setup()
{
  Serial.begin(115200);
  
  setupSPIFFS();
  setupLED();
  setupI2C();
  setupGPIO();

  initVolume();
  initAudio();
  initWiFi();
  initTMG();

  xTaskCreatePinnedToCore(playRadio, "radio", 5000, NULL, 5, &radioTask,0);
  xTaskCreate(showBattery, "battery", 5000, NULL, 1, &batteryTask);  
  xTaskCreate(processButtons, "keyb", 5000, NULL, 5, &keyboardTask);
}

void sleepWhileDark()
{
  int lightCount = -1;

  while (true)
  {
    if (tmg3993.getSTATUS() & STATUS_AVALID)
    {
      uint16_t r, g, b, c;
      int32_t lux;
  
      tmg3993.getRGBCRaw(&r, &g, &b, &c);
      lux = tmg3993.getLux(r, g, b, c);
  
      if (lux > luxThreshold)
      {
        // Light is on.
        if (darkCount > 0)
          darkCount = 0;

        // We are not sleeping?
        // No need to count...
        if (lightCount == -1)
          break;
    
        if (lightCount < lightDuration)
        {
          lightCount++;
          if (lightCount == lightDuration)
          {
            // Wake Up
            started = false;
            connected = false;

            adc_power_on();
            initWiFi();
            vTaskResume(radioTask);
            vTaskResume(batteryTask);
            vTaskResume(keyboardTask);
            ES8388_Write_Reg(25, 0x00);
            gpio_set_level(PA, 1);
            printf("Waking up.\n");
            break;
          }
        }
      }
      else
      {
        // Light is off.
        if (darkCount < darkDuration)
        {
          darkCount++;
          if (darkCount < darkDuration)
            break;

          lightCount = 0;
          
          // Go to Sleep
          printf("Going to sleep.\n");
          ES8388_Write_Reg(25, 0x04);
          vTaskSuspend(radioTask);
          vTaskSuspend(batteryTask);
          vTaskSuspend(keyboardTask);

          strip.SetPixelColor(0, BLACK);
          strip.Show();

          gpio_set_level(PA, 0);
          
          adc_power_off();
          WiFi.disconnect(true);
          WiFi.mode(WIFI_OFF);
        }
      }
    }
 
    // don't forget to clear the interrupt bits
    tmg3993.clearALSInterrupts();

    esp_sleep_enable_timer_wakeup(sleepDuration);
    esp_light_sleep_start();
  }

  tmg3993.clearALSInterrupts();
}

void sendMPDCmd(char *cmd)
{
  static WiFiClient client;

  client.connect(MPD_HOSTNAME, MPD_PORT);
  client.write(cmd);
  client.flush();
  client.stop();
}

void loop() {
  int oldVol;

  yield();

  // Check WiFi.
#ifdef SEARCH_STRONG_AP
  if (wifiMulti.run() != WL_CONNECTED)
#else
  if (WiFi.status() != WL_CONNECTED)
#endif
  {
    delay(100);
    return;
  }

  if (!started)
  {
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    started = true;
  }

  // Process volume buttons
  oldVol = vol;
  if (buttonPlus > 0 && buttonPlus < longKeypress)
  {
    vol++;
    buttonPlus = -1;
  }
  if (buttonMinus > 0 && buttonMinus < longKeypress)
  {
    vol--;
    buttonMinus = -1;
  }
  if (vol > maxVol)
    vol = maxVol;
  if (vol < 0)
    vol = 0;

  if (vol != oldVol)
  {
    char b[4];

    ES8388vol_Set(vol);

    printf("Volume: %d (max: %d)\n", vol, maxVol);

    sprintf(b,"%02d",vol);     
    File ln = SPIFFS.open("/volume", "w");
    ln.write((uint8_t*)b, 2);
    ln.close();     
  }
  
  // Process skip buttons
  if (buttonPlus > longKeypress)
  {
     sendMPDCmd("next\n");
     buttonPlus = -1;
  }
  if (buttonMinus > longKeypress)
  {
    sendMPDCmd("previous\n");
    buttonMinus = -1;
  }

  // Process mute button
  if (buttonPlay > 0 && buttonPlay < longKeypress)
  {
    mute = !mute;
    ES8388_Write_Reg(25, mute ? 0x04 : 0x00);

    buttonPlay = -1;
  } 

  // Deep sleep
  if(buttonPlay > longKeypress)
  {
    buttonPlay = -1;

    strip.SetPixelColor(0, BLACK);              //led off
    strip.Show();

    gpio_set_level(PA, 0);                      // power disable  

    delay(1000);
    esp_sleep_enable_ext0_wakeup(STOP,LOW);     // mute button to restart
    esp_deep_sleep_start();
  }

  delay(100);

  if (hasTMG)
    sleepWhileDark();
}

// Callbacks from Audio.h
void audio_info(const char *info)
{
  connected = (strstr(info, "failed") == NULL);
}

void audio_showstreamtitle(const char *info)
{
  Serial.print("Title: ");
  Serial.println(info);
}

void audio_id3data(const char *info){}
void audio_eof_mp3(const char *info){}
void audio_showstation(const char *info){}
void audio_showstreaminfo(const char *info){}
void audio_bitrate(const char *info){}
void audio_commercial(const char *info){}
void audio_icyurl(const char *info){}
void audio_lasthost(const char *info){}
void audio_eof_speech(const char *info){}
