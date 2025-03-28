// File:          dog_supervisor.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes

#include <webots/Supervisor.hpp>
#include <Eigen/Eigen>
#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <string>



// All the webots classes are defined in the "webots" namespace
using namespace std;
using namespace webots;
using namespace Eigen;

void ThreePoint(double* point, Block<MatrixXf> target, int i);
void fourPoint(double* point, Block<MatrixXf> target, int i);
void quaternionToAxisAngle(double* point, float x, float y, float z, float w);

int main(int argc, char **argv) {
  // create the Robot instance.
	Supervisor* robot = new Supervisor();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  string pointfilename = "lowjump_pointdata.txt";
  string targetfilename = "lowjump_targetpose.txt";
  int frames = 47;

  //string pointfilename = "midjump_pointdata.txt";
  //string targetfilename = "midjump_targetpose.txt";
  //int frames = 57;
  
  //string pointfilename = "heightjump_pointdata.txt";
  //string targetfilename = "heightjump_targetpose.txt";
  //int frames = 57;

  //string pointfilename = "ringjump1_pointdata.txt";
  //string targetfilename = "ringjump1_targetpose.txt";
  //int frames = 55;

  //string pointfilename = "ringjump2_pointdata.txt";
  //string targetfilename = "ringjump2_targetpose.txt";
  //int frames = 45;

  // import data
  ifstream file(pointfilename);
  if (!file.is_open()) {
      cerr << "�޷����ļ�" << endl;
      return -1;
  }

  int rows = frames, cols = 27;
  Eigen::MatrixXf matrix(rows, cols);
  string line, cell;
  int i = 0, j = 0;
  while (getline(file, line)) {
      stringstream ss(line);
      while (getline(ss,cell,',')) {
          matrix(i, j) = stof(cell);
          j++;
      }
      i++;
      j = 0;//��Ϊ���˰�������㣬һֱ���ӳ����������ֵ�Ĵ���
  }

  file.close();
  // ��ӡ����
  //cout << "��ȡ�ľ���Ϊ��" << endl << matrix << endl;
  //cout << "��ȡ�ľ�������Ϊ��" << matrix.rows() << "  " << matrix.cols() << endl;

  //��ȡgo2����������
  ifstream go2Pose(targetfilename);
  if (!go2Pose.is_open()) {
      cerr << "�޷����ļ�" << endl;
      return -1;
  }

  rows = frames;
  cols = 19;
  Eigen::MatrixXf go2matrix(rows, cols);

  i = 0;
  j = 0;
  while (getline(go2Pose, line)) {
      stringstream ss(line);

      while (getline(ss, cell, ',')) {

          go2matrix(i, j) = stof(cell);
          j++;
      }
      i++;
      j = 0;//��Ϊ���˰�������㣬һֱ���ӳ����������ֵ�Ĵ���
  }

  go2Pose.close();
// ��ӡ����
//cout << "��ȡ�ľ���Ϊ��" << endl << go2matrix << endl;
//cout << "��ȡ�ľ�������Ϊ��" << go2matrix.rows() << "  " << go2matrix.cols() << endl;

  //�������
  //��Ȯ����
  Block<MatrixXf> comS = matrix.block(0, 0, matrix.rows(), 3);

  Block<MatrixXf> hipRFS = matrix.block(0, 3, matrix.rows(), 3);
  Block<MatrixXf> footRFS = matrix.block(0, 6, matrix.rows(), 3);

  Block<MatrixXf> hipLFS = matrix.block(0, 9, matrix.rows(), 3);
  Block<MatrixXf>  footLFS = matrix.block(0, 12, matrix.rows(), 3);

  Block<MatrixXf> hipRHS = matrix.block(0, 15, matrix.rows(), 3);
  Block<MatrixXf> footRHS = matrix.block(0, 18, matrix.rows(), 3);

  Block<MatrixXf> hipLHS = matrix.block(0, 21, matrix.rows(), 3);
  Block<MatrixXf> footLHS = matrix.block(0, 24, matrix.rows(), 3);
  //cout << "��ȡ�ľ���Ϊ��" << endl << hipLHS << endl;
  //GO2����
  //Block<MatrixXf> go2Pos = go2matrix.block(0, 0, go2matrix.rows(), 3);
  //Block<MatrixXf> go2Rot = go2matrix.block(0, 3, go2matrix.rows(), 4);
  //Block<MatrixXf> go2RF = go2matrix.block(0, 7, go2matrix.rows(), 3);
  //Block<MatrixXf> go2LF = go2matrix.block(0, 10, go2matrix.rows(), 3);
  //Block<MatrixXf> go2RH = go2matrix.block(0, 13, go2matrix.rows(), 3);
  //Block<MatrixXf> go2LH = go2matrix.block(0, 16, go2matrix.rows(), 3);
  //cout << "��ȡ�ľ���Ϊ��" << endl << go2LH << endl;

  ///***go2����ο��켣***/
  nlohmann::json data;
  string danceFileName = "turn_and_jump.json";
//   string danceFileName = "swing.json";
  // string danceFileName = "wave.json";
//   string danceFileName = "trot.json";
  ifstream danceFile(danceFileName);
  try {
      data = nlohmann::json::parse(danceFile);
  }
  catch (const nlohmann::json::parse_error& e) {
      std::cerr << "��������: " << e.what() << std::endl;
      //return 1;
  }

    std::vector<std::vector<float>> matrix_data = data["frames"];
    size_t dance_rows = matrix_data.size();
    size_t dance_cols = matrix_data[0].size();
    MatrixXf dance_mat(dance_rows, dance_cols);
    for (size_t i = 0; i < dance_rows; ++i) {
        dance_mat.row(i) = Map<RowVectorXf>(matrix_data[i].data(), dance_cols);
    }

  Block<MatrixXf> go2Pos = dance_mat.block(0, 0, dance_mat.rows(), 3);
  Block<MatrixXf> go2Rot = dance_mat.block(0, 3, dance_mat.rows(), 4);
  Block<MatrixXf> go2RF = dance_mat.block(0, 25, dance_mat.rows(), 3);
  Block<MatrixXf> go2LF = dance_mat.block(0, 28, dance_mat.rows(), 3);
  Block<MatrixXf> go2RH = dance_mat.block(0, 31, dance_mat.rows(), 3);
  Block<MatrixXf> go2LH = dance_mat.block(0, 34, dance_mat.rows(), 3);

  // ����ڵ�
  Node* hipRF = robot->getFromDef("hipRF");
  Field* hipRFPos = hipRF->getField("translation");

  Node* footRF = robot->getFromDef("footRF");
  Field* footRFPos = footRF->getField("translation");

  Node* hipLF = robot->getFromDef("hipLF");
  Field* hipLFPos = hipLF->getField("translation");

  Node* footLF = robot->getFromDef("footLF");
  Field* footLFPos = footLF->getField("translation");

  Node* hipRH = robot->getFromDef("hipRH");
  Field* hipRHPos = hipRH->getField("translation");

  Node* footRH = robot->getFromDef("footRH");
  Field* footRHPos = footRH->getField("translation");

  Node* hipLH = robot->getFromDef("hipLH");
  Field* hipLHPos = hipLH->getField("translation");

  Node* footLH = robot->getFromDef("footLH");
  Field* footLHPos = footLH->getField("translation");

  Node* COM = robot->getFromDef("COM");
  Field* COMPos = COM->getField("translation");

  //GO2�ڵ�
  Node* GO2 = robot->getFromDef("GO2");
  Field* GO2ComPos = GO2->getField("translation");
  Field* GO2Rot = GO2->getField("rotation");

  Field* GO2HipRF = robot->getFromDef("HIPRF")->getField("position");
  Field* GO2HipLF = robot->getFromDef("HIPLF")->getField("position");
  Field* GO2HipRH = robot->getFromDef("HIPRH")->getField("position");
  Field* GO2HipLH = robot->getFromDef("HIPLH")->getField("position");
  //���������ùؽڽǶȵķ�ʽ����һ������setJointPosition�����ַ�ʽҪ��ȡ�ؽڽڵ�
  // �ڶ�����setSFFloat�����ַ�ʽҪ��ȡposition field
  // ��һ�ַ�ʽ���бȽ������ڶ��ַ�ʽ�����ٶȿ�
  //Node* GO2HipRF = robot->getFromDef("HIPRF");

  Field* GO2ThighRF = robot->getFromDef("thighRF")->getField("position");
  Field* GO2ThighLF = robot->getFromDef("thighLF")->getField("position");
  Field* GO2ThighRH = robot->getFromDef("thighRH")->getField("position");
  Field* GO2ThighLH = robot->getFromDef("thighLH")->getField("position");

  Field* GO2CalfRF = robot->getFromDef("calfRF")->getField("position");
  Field* GO2CalfLF = robot->getFromDef("calfLF")->getField("position");
  Field* GO2CalfRH = robot->getFromDef("calfRH")->getField("position");
  Field* GO2CalfLH = robot->getFromDef("calfLH")->getField("position");


  //GO2HipRF->setJointPosition(3, 1);
  double newpos[3] = { -1.04,-0.03,0.34 };
  double newrot[4] = { 0,0,0,0 };
  
  COMPos->setSFVec3f(newpos);
  GO2ComPos->setSFVec3f(newpos);
  const double* pos;

  while (robot->step(timeStep) != -1) {
      if (i >= dance_rows) {
          i = 0;
      }
      ////��Ȯ��λ
      //ThreePoint(newpos, hipRFS, i);
      //hipRFPos->setSFVec3f(newpos);

      //ThreePoint(newpos, footRFS, i);
      //footRFPos->setSFVec3f(newpos);
      ////cout << "��ǰ��С��" << *newpos << endl;
      //ThreePoint(newpos, hipLFS, i);
      //hipLFPos->setSFVec3f(newpos);

      //ThreePoint(newpos, footLFS, i);
      //footLFPos->setSFVec3f(newpos);

      //ThreePoint(newpos, hipRHS, i);
      //hipRHPos->setSFVec3f(newpos);

      //ThreePoint(newpos, footRHS, i);
      //footRHPos->setSFVec3f(newpos);

      //ThreePoint(newpos, hipLHS, i);
      //hipLHPos->setSFVec3f(newpos);

      //ThreePoint(newpos, footLHS, i);
      //footLHPos->setSFVec3f(newpos);

      //ThreePoint(newpos, comS, i);
      //COMPos->setSFVec3f(newpos);

      //GO2����λ��

      ThreePoint(newpos, go2Pos, i);
      GO2ComPos->setSFVec3f(newpos);
      //cout << "des pos��" << *newpos << " " << *(newpos + 1) << " " << *(newpos + 2) << endl;
      //pos = GO2->getPosition();
      //cout <<"real pos��" << *pos << " " << *(pos + 1) << " " << *(pos + 2) << endl;
      //cout << " " << endl;
      //fourPoint(newrot, go2Rot, i);
      quaternionToAxisAngle(newpos, go2Rot(i, 0), go2Rot(i, 1), go2Rot(i, 2), go2Rot(i, 3));
      GO2Rot->setSFRotation(newpos);
      //cout << "quat:" << *newrot << " " << *(newrot + 1) << " " << *(newrot + 2) << " " << *(newrot + 3) << endl;


      GO2HipRF->setSFFloat(go2RF(i, 0));
      GO2HipLF->setSFFloat(go2LF(i, 0));
      GO2HipRH->setSFFloat(go2RH(i, 0));
      GO2HipLH->setSFFloat(go2LH(i, 0));
      cout << "RF HIP:" << go2RF(i, 0) << " " << "LF HIP:" << go2LF(i, 0) << endl;

      GO2ThighRF->setSFFloat(go2RF(i, 1));
      GO2ThighLF->setSFFloat(go2LF(i, 1));
      GO2ThighRH->setSFFloat(go2RH(i, 1));
      GO2ThighLH->setSFFloat(go2LH(i, 1));
    //   cout << "RH THIGH:" << go2RH(i, 1) << " " << "LH THIGH:" << go2LH(i, 1) << endl;

      GO2CalfRF->setSFFloat(go2RF(i, 2));
      GO2CalfLF->setSFFloat(go2LF(i, 2));
      GO2CalfRH->setSFFloat(go2RH(i, 2));
      GO2CalfLH->setSFFloat(go2LH(i, 2));
    //   cout << "RH CLAF:" << go2RH(i, 2) << " " << "LH CLAF:" << go2LH(i, 2) << endl;

      i++;
       
  };



  delete robot;
  return 0;
}


void ThreePoint(double* point, Block<MatrixXf> target, int i) {
    point[0] = target(i, 0);
    point[1] = target(i, 1);
    point[2] = target(i, 2);
}

void fourPoint(double* point, Block<MatrixXf> target, int i) {
    point[0] = target(i, 3);
    point[1] = target(i, 0);
    point[2] = target(i, 1);
    point[3] = target(i, 2);
}

void quaternionToAxisAngle(double* point, float x, float y, float z, float w){
    // ������Ԫ����ע�����˳��Ϊ(w, x, y, z)
    Eigen::Quaterniond q(w, x, y, z);
    q.normalize(); // ȷ����λ��Ԫ��

    // ������ת�Ƕ�
    double angle = 2.0 * std::acos(q.w());

    // ������ֵ������acos����������Χ
    if (angle < 0) angle = 0.0;
    if (angle > 2 * M_PI) angle = 2 * M_PI;

    Eigen::Vector3d axis;
    const double epsilon = 1e-6;
    double sin_half_angle = std::sin(angle / 2.0);

    if (std::abs(sin_half_angle) > epsilon) {
        // �鲿��������sin(theta/2)�õ���λ��
        axis = q.vec() / sin_half_angle;
    }
    else {
        // ���Ƕ�������0��2��ʱ����δ���壬����Ĭ����
        axis = Eigen::Vector3d(1.0, 0.0, 0.0);
    }
    point[0] = axis.x();
    point[1] = axis.y();
    point[2] = axis.z();
    point[3] = angle;
}