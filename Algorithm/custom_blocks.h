#ifndef MISSION_CONTROL_CUSTOM_BLOCKS_H
#define MISSION_CONTROL_CUSTOM_BLOCKS_H

#include <sequence_ros.h>
#include "algorithm.h"

using namespace std;
using namespace seq;

namespace block_custom
{

class WallFollow : public block::SequenceBlock
{
public:
    WallFollow();

    string generateDebugName() override;
};

}
#endif //MISSION_CONTROL_CUSTOM_BLOCKS_H
