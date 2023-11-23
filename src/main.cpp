/**
 * Decode ADPCM from serial and send to I2S Amp
 * 
 * - #define COPY_LOG_OFF or change AudioLogger level to Warning to avoid hickups
 * - Adjust pin numbers in setup() according to your board
 * - Works fine with git main of ESP32 pio platform and arduino-audio-tools as of Nov 2023
 * 
 * have fun, JoBa-1
 */

#include <Arduino.h>

#include "AudioTools.h"
// #include "AudioCodecs/ContainerBinary.h"
// #include "AudioCodecs/CodecADPCM.h"
// can hang #include "AudioCodecs/CodecSBC.h"
// noise #include "AudioCodecs/CodecAPTX.h"
// silent errors #include "AudioCodecs/CodecLC3.h"


// Type of a channel sample, number of channels and sample rate
typedef int16_t in_chan_t;            // received sample size: 16 bits
typedef int16_t out_chan_t;           // MAX98357A can use 16bit
constexpr size_t InChans = 1;         // received channels
constexpr size_t OutChans = 2;        // sent channels, see MAX98357A datasheet
constexpr size_t SampleRate = 16000;  // recommended for MAX98357A


constexpr size_t InChanBytes = sizeof(in_chan_t);
constexpr size_t OutChanBytes = sizeof(out_chan_t);

constexpr size_t InChanBits = InChanBytes * CHAR_BIT;
constexpr size_t OutChanBits = OutChanBytes * CHAR_BIT;


AudioInfo in(SampleRate, InChans, InChanBits);
AudioInfo out(SampleRate, OutChans, OutChanBits);


/**
 * @brief Custom Stream 
 * someone reads from our out stream, which triggers us reading from our input stream
 * read 1ch 16bit values, write 2ch
 */
class Convert : public AudioStream {
public:
  Convert(Print &print) : _out(&print) {}

  void setStream(Stream &print) { _out = &print; }

  bool begin() {
    TRACEI();
    _read = 0;
    _last = 0;
    _written = 0;
    _maxval = 0;
    return _out != NULL;
  }

  void end() {
    TRACEI();
    _out = NULL;
  }

  /// amount of data available
  int availableForWrite() { return (((_out->availableForWrite() / OutChanBytes) * InChanBytes) / OutChans) * InChans; }

  /// Read and convert to out stream
  size_t write(const uint8_t *buffer, size_t size) override {
    // only write in chunks of in stream type
    size_t in_samples = size / InChanBytes / InChans;  // number of in samples
  
    // read at most as much as _out stream can handle
    size_t out_samples = _out->availableForWrite() / OutChanBytes / OutChans;
    if( out_samples < in_samples ) in_samples = out_samples;

    for( size_t i = 0; i < in_samples; i++ ) {
      for( size_t ch=0; ch < OutChans; ch++ ) {
        size_t i_ch = ch < InChans ? ch : InChans - 1;
        in_chan_t value = *(in_chan_t *)&buffer[(i*InChans+i_ch)*InChanBytes];
        out_chan_t val = (OutChanBits > InChanBits) ? (out_chan_t)value << (OutChanBits-InChanBits) : (out_chan_t)(value >> (InChanBits-OutChanBits));
        size_t written = _out->write((uint8_t *)&val, OutChanBytes);

        _written += written;

        if( written != OutChanBytes ) {
          LOGE("WRITE ERROR");
          return i * InChanBytes * InChans;
        }

        if( _maxval < val ) _maxval = val;
      }
    }

    _read += in_samples * InChanBytes * InChans;

    uint32_t now = millis();
    if( now - _last > _interval ) {
      _last = now;
      unsigned factor = (OutChanBytes*OutChans)/(InChanBytes*InChans);
      LOGI("Written/%u %u Bytes/s", factor, _written/factor);
      LOGI("Read      %u Bytes/s", _read);
      LOGI("Amplitude %u", _maxval);
      _written = 0;
      _read = 0;
      _maxval = 0;
    }

    return in_samples * InChanBytes * InChans;
  }

private:
  Print *_out;
  uint32_t _read;
  uint32_t _last;
  uint32_t _written;
  out_chan_t _maxval;

  static const uint32_t _interval = 1000;
};


I2SStream i2s;  // sink MAX98357A mono amp
Convert cvt(i2s);  // 1ch -> 2ch as required for the mono amp - go figure... :)
// BinaryContainerDecoder bcd(new ADPCMDecoder(AV_CODEC_ID_ADPCM_IMA_WAV));
// EncodedAudioStream dec(&cvt, &bcd);
// can hang EncodedAudioStream dec(&cvt, new BinaryContainerDecoder(new SBCDecoder()));
// noise EncodedAudioStream dec(&cvt, new BinaryContainerDecoder(new APTXDecoder()));
// EncodedAudioStream dec(&cvt, new BinaryContainerDecoder(new LC3Decoder()));
EncodedAudioStream dec(&cvt, new DecoderL8());
StreamCopy copier(dec, Serial1, 1024); 


void setup() {
  Serial.begin(BAUDRATE);
  AudioLogger::instance().begin(Serial, AudioLogger::Info);

  auto icfg = i2s.defaultConfig(TX_MODE);
  icfg.i2s_format = I2S_MSB_FORMAT;
  icfg.copyFrom(out);

  icfg.pin_data = 17;
  icfg.pin_ws = 21;
  icfg.pin_bck = 16;

  i2s.begin(icfg);
  dec.begin(in);
  // bcd.setAudioInfo(in);

  uint8_t rx=15;
  uint8_t tx=22;
  Serial1.begin(460800, SERIAL_8N1, rx, tx, false, 200);
  
  copier.begin();
}


void loop() {
  static const uint32_t Timeout = 1000;
  static uint32_t last_copy = 0;
  uint32_t now = millis();
  if( copier.copy() ) {
    last_copy = now;
  }
  else if( now - last_copy > Timeout ) {
    LOGE("Decoder restart")
    dec.clearWriteError();
    dec.decoder().end();
    dec.decoder().begin();
    last_copy = now;
  }
}
