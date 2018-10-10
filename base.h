#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <math.h>

#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD       // 直方体
#define dsDrawSphere   dsDrawSphereD    // 球
#define dsDrawCylinder dsDrawCylinderD  // 筒
#define dsDrawCapsule  dsDrawCapsuleD   // カプセル
#define dsDrawLine     dsDrawLineD      // 線
#endif

#define W 720         // グラフィックウィンドウの幅   Width
#define H 450         // グラフィックウィンドウの高さ Height

/*** シミュレータ形成 ***/
static dWorldID world;              // 動力学ワールド
static dSpaceID space;              // 衝突検出用スペース
static dGeomID ground;              // 地面
static dJointGroupID contactgroup;  // コンタクトグループ
dsFunctions fn;                     // ドロースタッフ用の構造体

/*** MyObject構造体 ***/
typedef struct {
  dBodyID body; // 剛体のID番号(動力学計算力)
  dGeomID geom; // ジオメトリIDの番号(動力学計算用)
  dReal w;      // 剛体の幅
  dReal h;      // 剛体の高さ
  dReal l;      // 剛体の長さ
  dReal r;      // 剛体の半径
  dReal m;      // 剛体の質量
} MyObject;

// ステップ
double one_step = 0.01;
double g = -9.81;

/***********************************/
/***** シミュレーション必須メソッド *****/
/***********************************/

/* コールバック関数(2つのジオメトリが衝突しそうなときに呼ばれる) */
static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  // 同時に衝突する可能性のある点数
  static const int N = 10;
  dContact contact[N];

  // 接触している物体のどちらかが地面ならisGroundを非0にする
  int isGround = ((ground == o1) || (ground == o2));

  // 2つの剛体がジョイントで結合されていたら衝突検出しない
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if(b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;

  // 衝突情報の生成
  int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
  // 地面以外の衝突を無効にする場合は下行をコメントアウト
  isGround = 1; // 地面
  if(isGround) {
    for(int i=0; i<n; i++) {
      contact[i].surface.mu = dInfinity;        // 摩擦係数
      contact[i].surface.mode = dContactBounce; // 接触面の反発性を設定
      contact[i].surface.bounce = 0.2;          // 反発係数
      contact[i].surface.bounce_vel = 0.0;      // 反発最低速度

      // 接触ジョイントの生成
      dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
      // 接触している2つの剛体を接触ジョイントにより拘束
      dJointAttach(c, dGeomGetBody(contact[i].geom.g1),
                      dGeomGetBody(contact[i].geom.g2));
    }
  }
}

void base_loop(int pause)
{
  if(!pause) {
    dSpaceCollide(space, 0, &nearCallback);
    dWorldStep(world, one_step);
    dJointGroupEmpty(contactgroup);
  }
}

/* 前処理 */
void start()
{
  /***** 視点とカメラの設定をする。 *****
   * xyz: 点(x, y, z)の位置に視線を設置
   * hpr: 視線の方向(heading, pitch, roll)を設定 *単位[deg]に注意
   * - h: heading x軸の旋回
   * - p: pitch   上下方向
   * - r: roll    左右の傾き
   */
  static float xyz[3] = {0.0, 2.0, 0.8};     // 視点の位置
  static float hpr[3] = {-90.0, 0.0, 0.0};   // 視点の方向
  dsSetViewpoint(xyz, hpr);                  // カメラを設定

  // 球は三角錐で形成する。その品質を設定。3で充分キレイ
  // 値が高すぎると描画に時間がかかる
  dsSetSphereQuality(3);                    // 球の品質を設定
}

/* 描画関数 */
void setDrawStuff()
{
  /*
   * 各パラメータに実装した関数のポインタを渡す。
   * 実装していない(ex: キー入力は受け付けない→command関数なし)ときは
   * nullを渡す(ex: fn.command = null)。
   * テクスチャのパスは注意 *間違っているとエラー出る。
   */
  fn.version = DS_VERSION;              // ドロースタッフのバージョン
  fn.start = &start;                    // 前処理(start関数)のポインタ
  fn.step = &simLoop;                   // simLoop関数のポインタ
  //fn.command = &command;                // command関数のポインタ
  fn.path_to_textures = "../textures";  // 読み込むテクスチャのパス
}

void base_main()
{
  setDrawStuff();                           // 描画関数の設定
  dInitODE();                               // ODEの初期化
  world = dWorldCreate();                   // 世界の創造
  dWorldSetGravity(world, 0, 0, g);         // 重力を展開

  space = dHashSpaceCreate(0);              // 衝突用空間を創造
  contactgroup = dJointGroupCreate(0);      // ジョイントグループ生成
  ground = dCreatePlane(space, 0, 0, 1, 0); // 地面を平面ジオメトリで生成
}

void finish_main(int argc, char *argv[])
{
  dsSimulationLoop(argc, argv, W, H, &fn);  // シミュレーションループ
  dSpaceDestroy(space);                     // スペースを破棄
  dWorldDestroy(world);                     // ワールドを破棄
  dCloseODE();                              // ODEの終了
}
