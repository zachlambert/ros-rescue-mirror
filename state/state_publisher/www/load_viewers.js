/**
 * Setup all visualization elements when the page is loaded.
 */
function init() {

    // Connect to ROS.
    // Specify the URL WebSocket URL for Rosbridge
    var ros = new ROSLIB.Ros({
        url : 'ws://localhost:9090'
    });

    console.log('Requesting topic list. If nothing is returned, there is a problem with connecting to rosbridge.');

    ros.getTopics(function callback(data){
        console.log(data.topics);
    });

    var global_viewer = new ROS3D.Viewer({
        divID: 'global_viewer',
        width: 600,
        height: 400,
        antialias: true
    });

    var local_viewer = new ROS3D.Viewer({
        divID: 'local_viewer',
        width: 600,
        height: 400,
        antialias: true
    })

    global_viewer.addObject(new ROS3D.Grid());
    // local_viewer.addObject(new ROS3D.Grid());

    // Setup a client to listen to global TFs.
    var global_tf_client = new ROSLIB.TFClient({
        ros : ros,
        fixedFrame : 'world',
        angularThres : 0.01,
        transThres : 0.01,
        rate : 10.0
    });

    // Setup a client to listen to local TFs.
    var local_tf_client = new ROSLIB.TFClient({
        ros : ros,
        fixedFrame : 'base_link',
        angularThres : 0.01,
        transThres : 0.01,
        rate : 10.0
    });

    // Setup URDF client for global view
    var urdfClient = new ROS3D.UrdfClient({
        ros : ros,
        tfClient : global_tf_client,
        path : 'http://localhost:8000/',
        rootObject : global_viewer.scene,
        loader : ROS3D.COLLADA_LOADER_2
    });

    // Setup URDF client for local view
    var urdfClient = new ROS3D.UrdfClient({
        ros : ros,
        tfClient : local_tf_client,
        path : 'http://localhost:8000/',
        rootObject : local_viewer.scene,
        loader : ROS3D.COLLADA_LOADER_2
    });

}
