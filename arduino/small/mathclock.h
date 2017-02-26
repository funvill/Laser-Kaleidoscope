/****************************************************************************** 

  Math clock
  Last updated: 2017-Feb-26
  Created by: Steven Smethurst @funvil 

  More information can be found here 
  https://github.com/funvill/mathclock

******************************************************************************/

static const unsigned short SETTING_MAX_NODES = 16;

class CMathClock
{
  private:
    
    float m_node[SETTING_MAX_NODES];

    // What the current times table is. 
    float m_timesTable;

    // How fast the system moves thought the times tables. 
    float m_movementRate; 
    

  public:
	  CMathClock() { this->Reset(); }
    void Reset();
    void Loop();
    float GetNodeAngle(unsigned short nodeID);
    float GetTimesTable() { return this->m_timesTable; }
    void SetMovementRate( float rate) { this->m_movementRate = rate ; }
};
extern CMathClock MathClock;
