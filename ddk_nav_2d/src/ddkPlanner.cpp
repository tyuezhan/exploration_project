#include <ddk_nav_2d/ddkPlanner.h>
#include <iostream>
#include <queue>
#include <vector>
#include <math.h>


ddkPlanner::ddkPlanner(){};

ddkPlanner::~ddkPlanner(){};


bool ddkPlanner::updateOctomap(octomap_msgs::Octomap msg){
    octomap::AbstractOcTree* tree = octomap_msgs::fullMsgToMap(msg);
    octoTree = dynamic_cast<octomap::OcTree*>(tree);
    resolution = octoTree->getResolution();
    octoTree->getMetricMin(minX, minY, minZ);
    octoTree->getMetricMax(maxX, maxY, maxZ);
    ROS_INFO("ddk planner Octomap Updated. Resolution: %f, minX: %f, minY: %f, minZ: %f, maxX: %f, maxY: %f, maxZ: %f", resolution, minX, minY, minZ, maxX, maxY, maxZ);
    // if (octoTree->search(0.001, 0.002, 0.003)){
    //     octomap::OcTreeNode* node;
    //     node = octoTree->search(0.001, 0.002, 0.003);
    //     ROS_INFO("exist, value: %f", node->getValue());

    // }else{
    //     ROS_INFO("Not exist");
    //     Vec3 currPos(0.001, 0.002, 0.003);
    //     Vec3 newPos = posInMap(currPos);
    //     ROS_INFO("New pos: %f, %f, %f", newPos(0), newPos(1), newPos(2));
    // }
    // octomap::OcTreeNode* node;
    // node = octoTree->search(0.001, 0.002, 0.003);
    // if (node==NULL) ROS_INFO("Node == NULL");

}

void ddkPlanner::displayData(){
    for (octomap::OcTree::leaf_iterator it = octoTree->begin_leafs(), end=octoTree->end_leafs(); it!=end; ++it){
        ROS_INFO("Node center: %f, %f, %f, %f",  it.getX(), it.getY(), it.getZ(), it->getValue());
    }
}


Vec3 ddkPlanner::posInMap(Vec3 pos){
    Vec3 mapPos;
    float x = pos(0);
    float y = pos(1);
    float z = pos(2);
    if ((x - std::floor(x / resolution) * resolution) <= (resolution / 2)){
        mapPos(0) = std::floor(x / resolution) * resolution;
    }else{
        mapPos(0) = std::ceil(x / resolution) * resolution;
    }

    if ((y - std::floor(y / resolution) * resolution) <= (resolution / 2)){
        mapPos(1) = std::floor(y / resolution) * resolution;
    }else{
        mapPos(1) = std::ceil(y / resolution) * resolution;
    }

    if ((z - std::floor(z / resolution) * resolution) <= (resolution / 2)){
        mapPos(2) = std::floor(z / resolution) * resolution;
    }else{
        mapPos(2) = std::ceil(z / resolution) * resolution;
    }
    return mapPos;
}


bool ddkPlanner::isFree(Vec3 pos){
    octomap::OcTreeNode* node;
    node = octoTree->search((double)pos(0), (double)pos(1), (double)pos(2));
    if (node == NULL){
        // Unknown
        return false;    
    }else{
        // known, check free/occupied
        if (octoTree->isNodeOccupied(node)){
            // Occupied
            return false;
        }
        else{
            return true;
        }
    }
}


bool ddkPlanner::isFrontier(Vec3 pos){
    octomap::OcTreeNode* node;
    if(isFree(pos)){
        // remove case that frontier continue to be z axis.
        if (pos(2) >= 0.5) return false;
        // check neighbors, if exist unknown
        Vec3 neighbors[26];
        getNeighbors(pos, neighbors);
        // ROS_INFO("Check if neighbor has unknown.");
        for (int i = 0; i < 26; i++){
            // ROS_INFO("Checking neighbor %d", i);
            node = octoTree->search((double)neighbors[i](0), (double)neighbors[i](1), (double)neighbors[i](2));
            if (node == NULL){
                ROS_INFO("Neighbor of pos %f, %f, %f is unknown", neighbors[i](0), neighbors[i](1), neighbors[i](2));
                return true;
            }
        }
        return false;
    }else{
        ROS_INFO("is frontier return false");
        return false;
    }
}

bool ddkPlanner::isInMap(Vec3 pos){
    // if (pos(0) < minX || pos(0) > maxX) return false;
    // if (pos(1) < minY || pos(1) > maxY) return false;
    if (pos(2) < 0 || pos(1) > 0.5) return false;
    return true;
}


void ddkPlanner::getNeighbors(Vec3 currentPos, Vec3* neighbors){
    // Vec3 neighbors[26];
    float x = currentPos(0);
    float y = currentPos(1);
    float z = currentPos(2);
    Vec3 neighbor0(x-resolution, y, z);
    Vec3 neighbor1(x+resolution, y, z);
    Vec3 neighbor2(x, y-resolution, z);
    Vec3 neighbor3(x, y+resolution, z);
    Vec3 neighbor4(x, y, z-resolution);
    Vec3 neighbor5(x, y, z+resolution);
    Vec3 neighbor6(x-resolution, y-resolution, z);
    Vec3 neighbor7(x+resolution, y-resolution, z);
    Vec3 neighbor8(x-resolution, y+resolution, z);
    Vec3 neighbor9(x+resolution, y+resolution, z);
    Vec3 neighbor10(x-resolution, y, z-resolution);
    Vec3 neighbor11(x+resolution, y, z-resolution);
    Vec3 neighbor12(x-resolution, y, z+resolution);
    Vec3 neighbor13(x+resolution, y, z+resolution);
    Vec3 neighbor14(x, y-resolution, z-resolution);
    Vec3 neighbor15(x, y+resolution, z-resolution);
    Vec3 neighbor16(x, y-resolution, z+resolution);
    Vec3 neighbor17(x, y+resolution, z+resolution);
    Vec3 neighbor18(x-resolution, y-resolution, z-resolution);
    Vec3 neighbor19(x+resolution, y-resolution, z-resolution);
    Vec3 neighbor20(x-resolution, y+resolution, z-resolution);
    Vec3 neighbor21(x-resolution, y-resolution, z+resolution);
    Vec3 neighbor22(x+resolution, y+resolution, z-resolution);
    Vec3 neighbor23(x+resolution, y-resolution, z+resolution);
    Vec3 neighbor24(x-resolution, y+resolution, z+resolution);
    Vec3 neighbor25(x+resolution, y+resolution, z+resolution);
    neighbors[0] = neighbor0;
    neighbors[1] = neighbor1;
    neighbors[2] = neighbor2;
    neighbors[3] = neighbor3;
    neighbors[4] = neighbor4;
    neighbors[5] = neighbor5;
    neighbors[6] = neighbor6;
    neighbors[7] = neighbor7;
    neighbors[8] = neighbor8;
    neighbors[9] = neighbor9;
    neighbors[10] = neighbor10;
    neighbors[11] = neighbor11;
    neighbors[12] = neighbor12;
    neighbors[13] = neighbor13;
    neighbors[14] = neighbor14;
    neighbors[15] = neighbor15;
    neighbors[16] = neighbor16;
    neighbors[17] = neighbor17;
    neighbors[18] = neighbor18;
    neighbors[19] = neighbor19;
    neighbors[20] = neighbor20;
    neighbors[21] = neighbor21;
    neighbors[22] = neighbor22;
    neighbors[23] = neighbor23;
    neighbors[24] = neighbor24;
    neighbors[25] = neighbor25;
    // return neighbors;
}


int ddkPlanner::findFrontierNN(Vec3 startPos, Vec3 &goalPos){
    ROS_INFO("start find frontiers.");
    std::queue<Vec3> queue;
    std::vector<Vec3> checkedPos;
    int checkCount = 0;
    bool foundFrontier = false;
    Vec3 startPosInMap = posInMap(startPos);
    queue.push(startPosInMap);
    checkedPos.push_back(startPosInMap);

    Vec3 currentPos;
    std::vector<Vec3>::iterator it;
    while (!queue.empty()){
        currentPos = queue.front();
        queue.pop();
        checkCount ++;
        //Check if it is frontier
        if (isFrontier(currentPos)){
            ROS_INFO("Found frontier.");
            goalPos = currentPos;
            foundFrontier = true;
            break;

        }else{
            Vec3 neighbors[26];
            getNeighbors(currentPos, neighbors);
            for (int i = 0; i < 26; i++){
                it = std::find(checkedPos.begin(), checkedPos.end(), neighbors[i]);
                if (it == checkedPos.end() && isFree(neighbors[i]) && isInMap(neighbors[i])){       
                    queue.push(neighbors[i]);
                    checkedPos.push_back(neighbors[i]);
                }
            }
        }
    }
    ROS_INFO("Finished find frontier loop.");
    if(foundFrontier){
        return EXPL_TARGET_SET;

    }else{
        if (checkCount > 50){
            return EXPL_FINISHED;
        }else{
            return EXPL_FAILED;
        }
    }

}


