// ROS headers
#include <ros/ros.h>

// MoveIt! headers
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>

// Std C++ headers
#include <string>
#include <vector>
#include <map>
#include <math.h> 

// Own pkgs
#include <tiago_move_arm_pylon/plan_arm_torso_ik_pylon.h>
#include <zone_ground_pylon/generate_circular_poi.h>
#include <tiago_reachable_box/ReachableBox.h>

using namespace tiago_move_arm_pylon;
using namespace zone_ground_pylon;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void go_to_point_base_footprint(move_base_msgs::MoveBaseGoal base_goal_pose_pylon_frame, MoveBaseClient& ac) {

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ROS_INFO("Sending goal");
  ac.sendGoal(base_goal_pose_pylon_frame);
}

geometry_msgs::PoseStamped transform(geometry_msgs::PoseStamped source_frame, std::string target_frameid) {
  ros::NodeHandle nh("~");
  double rate_hz;
  nh.param("rate", rate_hz, 1.0);
  ros::Rate rate(rate_hz);
  tf::TransformListener tf;
  geometry_msgs::PoseStamped goal_pose_frame;


  tf.waitForTransform(target_frameid, source_frame.header.frame_id, ros::Time(), ros::Duration(1.0));
   while(goal_pose_frame.header.frame_id != target_frameid)
  {
    try
    {
      //target_frame, stamped_in, stamped_out
      tf.transformPose(target_frameid, source_frame, goal_pose_frame);
    }
    catch(tf::TransformException& ex)
    {
      std::cout << "Failure at "<< ros::Time::now() << std::endl;
      std::cout << "Exception thrown:" << ex.what()<< std::endl;
    }
    rate.sleep();
  }
  return goal_pose_frame;
}

std::array<double,3> getPoseOT(std::string frame){

    // lecture de la position de l'OT 
    geometry_msgs::PoseStamped gripper_grasping_frame;
    gripper_grasping_frame.header.frame_id = "gripper_grasping_frame";
    gripper_grasping_frame.pose.position.x = 0;
    gripper_grasping_frame.pose.position.y = 0;
    gripper_grasping_frame.pose.position.z = 0;
    gripper_grasping_frame.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

    // tranforme de la position de l'OT dans le repere frame choisi en paramètre
    geometry_msgs::PoseStamped gripper_grasping_frame_base_footprint_frame = transform(gripper_grasping_frame, frame);

    std::array<double,3> pointOT = { { gripper_grasping_frame_base_footprint_frame.pose.position.x,
                                       gripper_grasping_frame_base_footprint_frame.pose.position.y,
                                       gripper_grasping_frame_base_footprint_frame.pose.position.z} };

    return pointOT;

    }


int main(int argc, char** argv)
{
  ros::init(argc, argv, "plan_arm_torso_ik_pylon");

  Move_tiago_arm tiago_arm;
  Generate_circular_poi gen_poi;
  MoveBaseClient ac("move_base", true);

  //tiago_arm.add_aptere();


  ros::NodeHandle n; // node hadler et publisher pour l'affichage des points de passage du bras 
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1); 
  // CHANGER LE NOM DU PUBLISHER POUR FAIRE UN AFFICHAGE PAR COUCHE DES POINTS 


  /*---------- LISTE DE POINTS DE PASSAGE -----------*/

  // definition de la forme du marker de passage 
  uint32_t formes = visualization_msgs::Marker::SPHERE_LIST;

  visualization_msgs::Marker pointsPassage;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  pointsPassage.header.frame_id = "/pylon";
  pointsPassage.header.stamp = ros::Time::now();

  // Set du namespace et id du marker.  ID unique
  // Attention si le meme namespace et id sont utilise, il y a ecrasement du precedent
  pointsPassage.ns = "follow_path";
  pointsPassage.id = 0;
  // set le type de marqueur a vec la forme 
  pointsPassage.type = formes; 

  // ajout du marqueur sous le format action 
  pointsPassage.action = visualization_msgs::Marker::ADD;

  //setup de la taille du marker sur chaque axe 
  pointsPassage.scale.x = 0.05;
  pointsPassage.scale.y = 0.05; 
  pointsPassage.scale.z = 0.05;
  // de la couleur (entre 0 et 1)
  pointsPassage.color.r = 0.0f;
  pointsPassage.color.g = 0.0f;
  pointsPassage.color.b = 1.0f; 
  pointsPassage.color.a = 1.0; // couche alpha, si a 0 --> 100% de transparence  

  // set de la duree du marker
  pointsPassage.lifetime = ros::Duration();

  /*---------- LISTE DE POINTS D APPROCHE BASE_FOOTPRINT -----------*/

  /* --------- MAINTENANT DANS PLOT_BROADCASTER --------------------*/

  /* ---------------------------------------------------------------*/

  ROS_INFO_STREAM("DEBUT DE FOLLOW PATH");

  //map_pose
  geometry_msgs::PoseStamped map_frame;
  map_frame.header.frame_id = "map";
  map_frame.pose.position.x = 0;
  map_frame.pose.position.y = 0;
  map_frame.pose.position.z = 0;
  map_frame.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);


  geometry_msgs::PoseStamped pylon_in_map_frame = transform(map_frame, "pylon");
  
	// read params from follow_path.launch
	ros::NodeHandle nh("~");
	int nb_pts_cercle = -1;
	nh.getParam("/nb_pts_base_footprint", nb_pts_cercle);
	
	int nb_pts_paths = -1;
	nh.getParam("/nb_pts_paths", nb_pts_paths);

	float rayon_cercle_base_footprint = -1.0;
	nh.getParam("/rayon_base_footprint", rayon_cercle_base_footprint);

	float rayon_cercle_arm = -1.0;
	nh.getParam("/rayon_arm", rayon_cercle_arm);
	
  
  //generation d'un cercle 
  ROS_INFO_STREAM("GENERATION DU CERCLE EN COURS ...");
  std::vector <std::vector<double> > cercle = gen_poi.generate_circle(nb_pts_cercle, rayon_cercle_base_footprint, 0, 0, 0, 0);


  //generation d'une trajectoire plus petite que 
  ROS_INFO_STREAM("GENERATION DU PATH EN COURS ...");
  std::vector <std::vector<double> > pathToFollow = gen_poi.generate_circle(nb_pts_paths, rayon_cercle_arm, 0, 0, 0, 0.3);

  // creation de la liste de spheres 
  //positionnement des markers dans le monde 
  for(uint32_t i = 0; i < nb_pts_paths; ++i){
    geometry_msgs::Point p;
    p.x = pathToFollow[0][i];
    p.y = pathToFollow[1][i];
    p.z = pathToFollow[2][i];
    pointsPassage.points.push_back(p);
  }
  ROS_INFO_STREAM("GENERATION DU PATH FAITE");

   // -----------------------AFFICHAGE ---------------------------------- //
  // publication des markers 
  while (marker_pub.getNumSubscribers() < 1){
    if (!ros::ok()) {
      return 0;
    }
    ROS_WARN_ONCE("Dans Rviz ajouter un marker pour visualiser les points");
    sleep(1);
  }
  marker_pub.publish(pointsPassage); //publication de la liste des points de passage
 // marker_pub.publish(pointsApprocheBaseFootprint); //publication de la liste des points de passage
  // ---------------------- FIN AFFICHAGE ----------------------------- // 

  //std::array<std::array<double, 3>, nb_pts_paths> listePointOrigine; // on set une liste de la taille du nombre de points a atteindre
  //std::array<std::array<double, 3>, nb_pts_paths> listePointGoal; 
  
  std::vector<std::array<double, 3>> listePointOrigine; // on set une liste de la taille du nombre de points a atteindre
  std::vector<std::array<double, 3>> listePointGoal; 
  
  // creation de la liste qui contiendra les chemins 
  std::vector<std::vector <std::vector<double> > > listeChemin;

  // declaration de la structure de position du bras
  geometry_msgs::PoseStamped arm_goal_pose;

  // etape 0 : Planifier pour chaque point de passage la trajectoire associée
  for(int i = 0; i < nb_pts_paths; i++){

    std::cout << "chemin " << i <<  "de la liste chemin " << std::endl; 

    if(i == 0){ // si on travaille sur le premier point, on se base par rapport à la position actuelle de tiago 

      geometry_msgs::PoseStamped base_footprint_frame;
      base_footprint_frame.header.frame_id = "base_footprint";
      base_footprint_frame.pose.position.x = 0;
      base_footprint_frame.pose.position.y = 0;
      base_footprint_frame.pose.position.z = 0;
      base_footprint_frame.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

      geometry_msgs::PoseStamped base_footprint_pylon_frame = transform(base_footprint_frame, "pylon");
			std::array<double, 3> pts_origine;
			pts_origine[0] = pathToFollow[0][i];
			pts_origine[1] = pathToFollow[1][i];
			pts_origine[2] = pathToFollow[2][i];
			listePointOrigine.push_back(pts_origine);
      //listePointOrigine[i][0] = pathToFollow[0][i];
      //listePointOrigine[i][1] = pathToFollow[1][i];
      //listePointOrigine[i][2] = pathToFollow[2][i];
    }
    else{
      std::cout << "Dans le ELSE" << std::endl; 
      listePointOrigine.push_back(listePointGoal[i-1]); // sinon on considere le dernier point atteint comme le nouveau point de depart de l'algo (chemin)
    }
    
    // on calcule ensuite le point suivant
    //listePointGoal[i][0] = pathToFollow[0][i];
    //listePointGoal[i][1] = pathToFollow[1][i];
    //listePointGoal[i][2] = pathToFollow[2][i];
		std::array<double, 3> pts_goal;
		pts_goal[0] = pathToFollow[0][i];
		pts_goal[1] = pathToFollow[1][i];
		pts_goal[2] = pathToFollow[2][i];
		listePointGoal.push_back(pts_goal);

    std::vector <std::vector<double> > chemin = gen_poi.generate_traj_2pt_from_circle(cercle,listePointOrigine[i],listePointGoal[i]);
    std::cout << "JE VIENS DE GENERER UN CHEMIN " << std::endl; 


    for (int k = 0; k < chemin[0].size(); k++){ // visu du chemin 
      std::cout << chemin[0][k] << std::endl << chemin[1][k] << std::endl; 
    }

    listeChemin.push_back(chemin); // ajout du chemin à la queue de la liste de chemin
    std::cout << "J AI APPEND !! " << std::endl; 

    // @TODO -----> tester a partir d'ici si la liste de chemin permet de rester atteignable en permanence
    
  }


  //std::vector <std::vector<double> > chemin = gen_poi.generate_traj_2pt_from_circle(cercle,pointOrigine,pointGoal);

  ROS_INFO_STREAM("/* Après génération des chemins  */");
  std::cout << "affichage du début des chemins" << std::endl;
  
  int i;
  tfScalar xx = 0; tfScalar xy = 1; tfScalar xz = 0;
  tfScalar yx = 1; tfScalar yy = 0; tfScalar yz = 0;
  tfScalar zx = 0; tfScalar zy = 0; tfScalar zz = -1;


  tf::Matrix3x3 matriceRotation(xx,xy,xz,yx,yy,yz,zx,zy,zz);

  tfScalar roll, pitch, yaw;

  geometry_msgs::PoseStamped arm_goal_in_pylon_frame;
  geometry_msgs::PoseStamped arm_goal_in_plot_frame;
  geometry_msgs::PoseStamped arm_goal_in_base_footprint_frame;

  matriceRotation.getRPY(roll,pitch,yaw);
  
  for (int numChemin = 0; numChemin < nb_pts_paths; numChemin++){ // pour tout les chemins générés 

    // on définit la variable de but de la base 
    move_base_msgs::MoveBaseGoal base_goal_pose_pylon_frame; 

    // on pick le chemin suivant entre deux points de passage
    std::vector <std::vector<double> > chemin = listeChemin[numChemin]; 
    
    for(i = 0; i < chemin[0].size(); i++) { // pour chemin enter deux points de passage

     std::cout << "\n\n x: "  << chemin[0][i] << " y: " << chemin[1][i] << " z: " << chemin[2][i] << "\n\n" << std::endl;

     // On cherche le repère du goal

     base_goal_pose_pylon_frame.target_pose.header.frame_id = "plot_" + std::to_string((int) chemin[3][0]); //chemin[3][0] donne l'indice du point de sortie du chemin, voir generate_circular_point

      // on va à l'origine du nouveau repère
      base_goal_pose_pylon_frame.target_pose.pose.position.x = 0; 
      base_goal_pose_pylon_frame.target_pose.pose.position.y = 0;
      base_goal_pose_pylon_frame.target_pose.pose.position.z = 0;
      
      // déplacement de la base mobile aux coordonnées but avec attente de réussite de déplacement
      // @TODO verifier que la plannif puisse toujours reussir.
      base_goal_pose_pylon_frame.target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
      go_to_point_base_footprint(base_goal_pose_pylon_frame,ac);
      //ac.waitForResult();
      ROS_INFO_STREAM("\n\n STATE ACTION DEPLACEMENT BASE : " + ac.getState().toString() + "\n\n");

    }
    
    // @TODO positionnner l'organe terminal SUR le waypoint
    // @TODO OT avec l'orientation adequate
    if(chemin[0].size() == 0 )
      std::cout << "Le chemin est vide ATTENTION je fais des betises pour le chemin " << numChemin  << std::endl;
    
    
    //Récupération actuelle de la position du bras (pour pouvoir trouver la bonne orientation a donné pour orienter le système aptère)
  
    //// définition de l'objectif a atteindre en fin de mouvement de la base mobile

    //étape 1 : Récupération de la position du point à atteindre dans le repère du pylone
    
    arm_goal_in_pylon_frame.header.frame_id = "pylon";
    arm_goal_in_pylon_frame.pose.position.x = pathToFollow[0][numChemin];
    arm_goal_in_pylon_frame.pose.position.y = pathToFollow[1][numChemin];
    arm_goal_in_pylon_frame.pose.position.z = pathToFollow[2][numChemin];
    arm_goal_in_pylon_frame.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

    //étape 2 transformation du point dans le repère du plot but
   
    ROS_INFO_STREAM("Demande de transformation frame");
    //TODO a optimiser
    arm_goal_in_plot_frame = transform(arm_goal_in_pylon_frame,"plot_"+std::to_string((int) chemin[3][0]));
    ROS_INFO_STREAM("Transformation effectuée");
    arm_goal_in_plot_frame.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);

    //étape 3 copie des coordonées du point dans le repère du plot dans le repère de la base_footprint (CE N'EST PAS UNE TRANSFORMATION !)
    
    arm_goal_in_base_footprint_frame = arm_goal_in_plot_frame;

    arm_goal_in_base_footprint_frame.header.frame_id = "base_footprint";

    // début du mouvement pour le bras
    ROS_INFO_STREAM("ENVOIE DE L'ORDRE AU BRAS");
    //tiago_arm.go_to_point_arm_tool_link(arm_goal_in_base_footprint_frame);
  
  }

  /*---------- Visualisation de la boite englobante -----------*/

  ROS_INFO_STREAM("GENERATION DE LA BOX DE COLLISION ... EN COURS");

  geometry_msgs::PoseStamped box_reachable;
  box_reachable.header.frame_id = "pylon";
  box_reachable.pose.position.x = 0.0;
  box_reachable.pose.position.y = 0.0;
  box_reachable.pose.position.z = 0.9;
  box_reachable.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
  
  ReachableBox boiteTest(box_reachable.pose.position.x,box_reachable.pose.position.y,box_reachable.pose.position.z,1.7f,1.7f,1.9f); // equivalent de la sphere en marker 

  geometry_msgs::PoseStamped point_to_test;
  point_to_test.header.frame_id = "pylon";
  point_to_test.pose.position.x = 2.25;
  point_to_test.pose.position.y = 0.0;
  point_to_test.pose.position.z = 0.0;
  point_to_test.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

  geometry_msgs::PoseStamped point_arm_1_link = transform(point_to_test,"arm_1_link");

  if(boiteTest.isReachable(point_to_test.pose.position.x,point_to_test.pose.position.y,point_to_test.pose.position.z) == true)
    std::cout <<  "Une collision existe" << std::endl;
  else
    std::cout << "Aucune collision" << std::endl;
  
  
  ROS_INFO_STREAM("Le programme du suivi de chemin s'est terminé avec un franc succés ! Félicitations !");
  return 0;

}
