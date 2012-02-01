
namespace Andor
{
  class Camera
  {
%TypeHeaderCode
#include <AndorCamera.h>
%End

  public:

    enum Status {
      Ready, Exposure, Readout, Latency, Fault
    };

    enum ShutterMode{
      FRAME,
      MANUAL
    };

    Camera(const std::string& config_path,int camera_number=0);
    ~Camera();

    void startAcq();
    void stopAcq();

    void getImageType(ImageType& type /Out/);
    void setImageType(ImageType type);

    void getDetectorType(std::string& type /Out/);
    void getDetectorModel(std::string& model /Out/);
    void getDetectorImageSize(Size& size /Out/);
    HwBufferCtrlObj* getBufferMgr();

    void setTrigMode(TrigMode  mode);
    void getTrigMode(TrigMode& mode /Out/);
	
    void setExpTime(double  exp_time);
    void getExpTime(double& exp_time /Out/);

    void setLatTime(double  lat_time);
    void getLatTime(double& lat_time /Out/);

    void getExposureTimeRange(double& min_expo /Out/, double& max_expo /Out/);
    void getLatTimeRange(double& min_lat /Out/, double& max_lat /Out/);

    void setNbFrames(int  nb_frames);
    void getNbFrames(int& nb_frames /Out/);

    void checkRoi(const Roi& set_roi, Roi& hw_roi /Out/);
    void setRoi(const Roi& set_roi);
    void getRoi(Roi& hw_roi /Out/);

    void checkBin(Bin& /In,Out/);
    void setBin(const Bin&);
    void getBin(Bin& /Out/);

    void setShutterOpenTime(double tm);
    void getShutterOpenTime(double& tm /Out/);
    void setShutterCloseTime(double tm);
    void getShutterCloseTime(double& tm /Out/);
    void setShutter(bool flag);
    void getShutter(bool& flag /Out/);
    void setShutterMode(ShutterMode mode);
    void getShutterMode(ShutterMode& mode /Out/);

    void getPixelSize(double& xsize /Out/, double& ysize /Out/);

    void getStatus(Andor::Camera::Status& status /Out/);

    void initialiseController();
    void initAdcSpeed();
    void setAdcSpeed(int adc);
    void initVSS();
    void setVSS(int vss);
    void initPGain();
    void setPGain(int gain);
    void setFastExtTrigger(bool flag);
    void getFastExtTrigger(bool& flag /Out/);
    void setShutterLevel(int level);
    void getShutterLevel(int& level /Out/);
    void setTemperatureSP(int temp);
    void getTemperatureSP(int& temp);
    void getTemperature(int& temp /Out/);
    void setCooler(bool flag);
    void getCooler(bool& flag /Out/);
    void getCoolingStatus(std::string& status /Out/);
  };

};
