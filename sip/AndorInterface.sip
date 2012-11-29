namespace Andor
{
  class Interface : HwInterface
  {
%TypeHeaderCode
#include <AndorInterface.h>
%End
  public:


    Interface(Andor::Camera& cam);
    virtual ~Interface();

    //- From HwInterface
    //    virtual void 	getCapList(CapList& /Out/) const;
    virtual void	getCapList(std::vector<HwCap> &cap_list /Out/) const;
    virtual void	reset(ResetLevel reset_level);
    virtual void 	prepareAcq();
    virtual void 	startAcq();
    virtual void 	stopAcq();
    virtual void 	getStatus(StatusType& status /Out/);
    virtual int 	getNbHwAcquiredFrames();

    void setAdcSpeed(int adc);
    void getAdcSpeed(int& adc /Out/);
    void setVsSpeed(int vss);
    void getVsSpeed(int& vss /Out/);
    void setPGain(int gain);
    void getPGain(int& gain /Out/);
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
    void setSpooling(bool flag, Andor::SpoolingMethod method, std::string path, int frameBufferSize);
    void setHighCapacity(Andor::HighCapacityMode mode);
    void setGateMode(Andor::GateMode mode);
  };
};
