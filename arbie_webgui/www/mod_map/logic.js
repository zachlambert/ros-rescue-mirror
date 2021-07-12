 function map_init() {

    
    // Connect to ROS.
    var ros = new ROSLIB.Ros({
        url: 'ws://' + window.location.hostname + ':9090'
    });

    var robot_coord = {
        x:0,
        y:0,
        z:0,
        w:0
    };
    var goal_coord = {
        x:0,
        y:0,
        z:0,
        w:0
    };

    var goal_reached_Topic = new ROSLIB.Topic({
      ros : ros,
      name : '/goal_reached',
      messageType : 'std_msgs/Bool'
    });


    ROS3D.Pose = function(options) {
      THREE.Object3D.call(this);
      this.options = options || {};
      this.ros = options.ros;
      this.topicName = options.topic || '/pose';
      this.tfClient = options.tfClient;
      this.color = options.color || 0xcc00ff;
      this.rootObject = options.rootObject || new THREE.Object3D();
      this.sn = null;
      this.rosTopic = undefined;
      this.subscribe();
    };
    ROS3D.Pose.prototype.__proto__ = THREE.Object3D.prototype;
    ROS3D.Pose.prototype.unsubscribe = function(){
      if(this.rosTopic){
        this.rosTopic.unsubscribe();
      }
    };
    ROS3D.Pose.prototype.subscribe = function(){
      this.unsubscribe();
      // subscribe to the topic
      this.rosTopic = new ROSLIB.Topic({
          ros : this.ros,
          name : this.topicName,
          queue_length : 1,
          messageType : 'geometry_msgs/PoseStamped'
      });
      this.rosTopic.subscribe(this.processMessage.bind(this));
    };
    ROS3D.Pose.prototype.processMessage = function(message){
      if(this.sn!==null){
          this.sn.unsubscribeTf();
          this.rootObject.remove(this.sn);
      }
      this.options.origin = new THREE.Vector3( message.pose.position.x, message.pose.position.y,
                                               message.pose.position.z);
      var rot = new THREE.Quaternion(message.pose.orientation.x, message.pose.orientation.y,
                                     message.pose.orientation.z, message.pose.orientation.w);
      this.options.direction = new THREE.Vector3(1,0,0);
      this.options.direction.applyQuaternion(rot);
      this.options.material = new THREE.MeshBasicMaterial({color: this.color});
      var arrow = new ROS3D.Arrow(this.options);
      this.sn = new ROS3D.SceneNode({
          frameID : message.header.frame_id,
          tfClient : this.tfClient,
          object : arrow
      });
      this.rootObject.add(this.sn);
    };

    //redefine class: Point
    ROS3D.Point = function(options) {
        THREE.Object3D.call(this);
        this.options = options || {};
        this.ros = options.ros;
        this.topicName = options.topic || '/move_base_simple/goal';
        this.tfClient = options.tfClient;
        this.color = options.color || 0xcc00ff;
        this.rootObject = options.rootObject || new THREE.Object3D();
        this.radius = options.radius || 0.2;
        this.sn = null;
        this.rosTopic = undefined;
        this.subscribe();
      };
      ROS3D.Point.prototype.__proto__ = THREE.Object3D.prototype;
      ROS3D.Point.prototype.unsubscribe = function(){
        if(this.rosTopic){
          this.rosTopic.unsubscribe();
        }
      };
      ROS3D.Point.prototype.subscribe = function(){
        this.unsubscribe();
        // subscribe to the topic
        this.rosTopic = new ROSLIB.Topic({
            ros : this.ros,
            name : this.topicName,
            queue_length : 1,
            messageType : 'geometry_msgs/PoseStamped'
        });
        this.rosTopic.subscribe(this.processMessage.bind(this));
      };
      ROS3D.Point.prototype.processMessage = function(message){
        if(this.sn!==null){
            this.sn.unsubscribeTf();
            this.rootObject.remove(this.sn);
        }

          var sphereGeometry = new THREE.SphereGeometry( this.radius );
          var sphereMaterial = new THREE.MeshBasicMaterial( {color: this.color} );
          var sphere = new THREE.Mesh(sphereGeometry, sphereMaterial);
          sphere.position.set(message.pose.position.x, message.pose.position.y, message.pose.position.z);
          goal_coord = {x:message.pose.position.x, y:message.pose.position.y, z:message.pose.position.z, w:message.pose.orientation.w};

          this.sn = new ROS3D.SceneNode({
              frameID : message.header.frame_id,
              tfClient : this.tfClient,
              object : sphere
          });
          this.rootObject.add(this.sn);
      
      };

    //redefine class: occupancygridclient
    ROS3D.OccupancyGridClient = function(options) {
        EventEmitter2.call(this);
        options = options || {};
        this.ros = options.ros;
        this.topicName = options.topic || '/map';
        this.compression = options.compression || 'cbor';
        this.continuous = options.continuous;
        this.tfClient = options.tfClient;
        this.rootObject = options.rootObject || new THREE.Object3D();
        this.offsetPose = options.offsetPose || new ROSLIB.Pose();
        this.color = options.color || {r:255,g:255,b:255};
        this.opacity = options.opacity || 1.0;
        // current grid that is displayed
        this.currentGrid = null;
        // subscribe to the topic
        this.rosTopic = undefined;
        this.subscribe();
      };
      ROS3D.OccupancyGridClient.prototype.__proto__ = EventEmitter2.prototype;
      ROS3D.OccupancyGridClient.prototype.unsubscribe = function(){
        if(this.rosTopic){
          this.rosTopic.unsubscribe();
        }
      };
      ROS3D.OccupancyGridClient.prototype.subscribe = function(){

        // subscribe to the topic
        this.rosTopic = new ROSLIB.Topic({
          ros : this.ros,
          name : this.topicName,
          messageType : 'nav_msgs/OccupancyGrid',
          queue_length : 1,
          compression : this.compression
        });
        this.rosTopic.subscribe(this.processMessage.bind(this));
      };
      ROS3D.OccupancyGridClient.prototype.processMessage = function(message){
        // check for an old map
        if (this.currentGrid) {
          // check if it there is a tf client
          if (this.currentGrid.tfClient) {
            // grid is of type ROS3D.SceneNode
            this.currentGrid.unsubscribeTf();
          }
          this.rootObject.remove(this.currentGrid);
        }
        var newGrid = new ROS3D.OccupancyGrid({
          message : message,
          color : this.color,
          opacity : this.opacity
        });
        // check if we care about the scene
        if (this.tfClient) {
          this.currentGrid = newGrid;
          this.sceneNode = new ROS3D.SceneNode({
            frameID : message.header.frame_id,
            tfClient : this.tfClient,
            object : newGrid,
            pose : this.offsetPose
          });
        } else {
          this.sceneNode = this.currentGrid = newGrid;
        }
        this.rootObject.add(this.sceneNode);
        this.emit('change');
        // check if we should unsubscribe
        if (!this.continuous) {
          this.rosTopic.unsubscribe();
        }
      };
    
    var viewer = new ROS3D.Viewer({
        divID: 'map',
        width: 1000,
        height: 1000,
        antialias: true
    });

    console.log("map initialised")

    viewer.addObject(new ROS3D.Grid());
    // local_viewer.addObject(new ROS3D.Grid()); Don't have a grid on the local view

    // Setup a client to listen to global TFs.
    var global_tf_client = new ROSLIB.TFClient({
        ros : ros,
        fixedFrame : 'map',
        angularThres : 0.01,
        transThres : 0.01,
        rate : 10.0
    });

    // Setup URDF client for global view
    var urdfClient = new ROS3D.UrdfClient({
        ros : ros,
        tfClient : global_tf_client,
        path : 'http://' + window.location.hostname + ':8000/',
        rootObject : viewer.scene,
        loader : ROS3D.COLLADA_LOADER
    });


    var goalClient = new ROS3D.Point({
        ros : ros,
        fixedFrame : 'map',
        tfClient : global_tf_client,
        topic : '/move_base_simple/goal',
        rootObject : viewer.scene
    });

    var navClient = new ROS3D.Pose({
      ros : ros,
      fixedFrame : 'map',
      tfClient : global_tf_client,
      topic : '/navigation/goal',
      rootObject : viewer.scene
  });


    ROS3D.SceneNode = function(options) {
      THREE.Object3D.call(this);
      options = options || {};
      var that = this;
      this.tfClient = options.tfClient;
      this.frameID = options.frameID;
      var object = options.object;
      this.pose = options.pose || new ROSLIB.Pose();
      // Do not render this object until we receive a TF update
      this.visible = false;
      // add the model
      this.add(object);
      // set the inital pose
      this.updatePose(this.pose);
      // save the TF handler so we can remove it later
      this.tfUpdate = function(msg) {
        // apply the transform
        var tf = new ROSLIB.Transform(msg);
        var poseTransformed = new ROSLIB.Pose(that.pose);
        poseTransformed.applyTransform(tf);
        // update the world
        that.updatePose(poseTransformed);
        that.visible = true;
      };
      // listen for TF updates
      this.tfClient.subscribe(this.frameID, this.tfUpdate);
    };
    ROS3D.SceneNode.prototype.__proto__ = THREE.Object3D.prototype;
    /**
     * Set the pose of the associated model.
     *
     * @param pose - the pose to update with
     */
    ROS3D.SceneNode.prototype.updatePose = function(pose) {
      robot_coord = {x:pose.position.x, y:pose.position.y, z:pose.position.z, w:pose.orientation.w};

      if(goal_coord.x != 0 && goal_coord.y != 0){

        if(Math.abs(goal_coord.x-robot_coord.x)<0.2&&Math.abs(goal_coord.y-robot_coord.y)<0.2&&Math.abs(goal_coord.z-robot_coord.z)<1&&Math.abs(goal_coord.w-robot_coord.w)<1){
          var goal_reached_Message = new ROSLIB.Message({
            data : true
          });
        }
        else{
          var goal_reached_Message = new ROSLIB.Message({
            data : false
        });
        }
          goal_reached_Topic.publish(goal_reached_Message);
          console.log("Goal Reached");
        
      }


      this.position.set( pose.position.x, pose.position.y, pose.position.z );
      this.quaternion.set(pose.orientation.x, pose.orientation.y,
          pose.orientation.z, pose.orientation.w);
      this.updateMatrixWorld(true);
    };
    ROS3D.SceneNode.prototype.unsubscribeTf = function() {
      this.tfClient.unsubscribe(this.frameID, this.tfUpdate);
    };


    var arrowNode = new ROS3D.SceneNode({
      tfClient : global_tf_client,
      frameID  : '/base_link',   
      object   : new ROS3D.Arrow(),   
    });

  }