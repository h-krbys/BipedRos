/*
    author: H. Kuribayashi

    点や線などの図形をRVizに描画
 */

#ifndef __VALKYRIE_JOINT_CONDIG_H__
#define __VALKYRIE_JOINT_CONDIG_H__

#define NUM_JOINTS 43

class ValkyrieJointConfig {
public:
  ValkyrieJointConfig();
  ~ValkyrieJointConfig();

  int getJointId(int cnoid_id);
  int getName(int cnoid_id);

private:
  int         jointId[NUM_JOINTS];
  std::string name[NUM_JOINTS];
};

#endif