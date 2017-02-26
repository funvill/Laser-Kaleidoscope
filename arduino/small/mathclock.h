


static const unsigned short SETTING_MAX_NODES = 16;
static const float SETTING_MOVEMENT_RATE = 1.0f;

class CMathClock
{
  private:
    float m_node[SETTING_MAX_NODES];
    float m_timesTable;

  public:
    void Reset();
    void Loop();
    float GetNodeAngle(unsigned short nodeID);
};
extern CMathClock MathClock;
