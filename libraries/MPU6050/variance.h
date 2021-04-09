#ifndef VARIANCE_H
#define VARIANCE_H

class variance {
  public:
    variance(void) {};
    void reset(void);
    void update(short* reading);
    unsigned long getSum(void);

  private:
    unsigned short threshold = 20;
    unsigned long sum[3];
    unsigned long sqsum[3];
    unsigned long var[3];  //variance
    //unsigned long sd[3]; //standard deviation
    unsigned short dataPoints;
};


#endif
