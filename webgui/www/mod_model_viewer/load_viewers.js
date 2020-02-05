/**
 * Setup all visualization elements when the page is loaded.
 */
function init() {

    // Connect to ROS.
    var ros = new ROSLIB.Ros({
        url : 'ws://localhost:9090'
    });

    var global_viewer = new ROS3D.Viewer({
        divID: 'global_viewer',
        width: 1000,
        height: 1000,
        antialias: true
    });

    // var local_viewer = new ROS3D.Viewer({
    //     divID: 'local_viewer',
    //     width: 600,
    //     height: 400,
    //     antialias: true
    // })

    global_viewer.addObject(new ROS3D.Grid());
    // local_viewer.addObject(new ROS3D.Grid()); Don't have a grid on the local view

    // Setup a client to listen to global TFs.
    var global_tf_client = new ROSLIB.TFClient({
        ros : ros,
        fixedFrame : 'world',
        angularThres : 0.01,
        transThres : 0.01,
        rate : 10.0
    });

    // // Setup a client to listen to local TFs.
    // var local_tf_client = new ROSLIB.TFClient({
    //     ros : ros,
    //     fixedFrame : 'base_link',
    //     angularThres : 0.01,
    //     transThres : 0.01,
    //     rate : 10.0
    // });

    // Setup URDF client for global view
    var urdfClient = new ROS3D.UrdfClient({
        ros : ros,
        tfClient : global_tf_client,
        path : 'http://localhost:8000/',
        rootObject : global_viewer.scene,
        loader : ROS3D.COLLADA_LOADER_2
    });

    // // Setup URDF client for global view
    // var urdfClient = new ROS3D.UrdfClient({
    //     ros : ros,
    //     tfClient : local_tf_client,
    //     path : 'http://localhost/',
    //     rootObject : local_viewer.scene,
    //     loader : ROS3D.COLLADA_LOADER_2
    // });

}
