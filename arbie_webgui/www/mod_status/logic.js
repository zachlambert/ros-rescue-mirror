decayer = null;

function decayOpacity() {
    let l = document.getElementById("pingpong_status_light");
    l.style.opacity = String(Number(l.style.opacity) - 0.05 > 0 ? Number(l.style.opacity) - 0.05 : 0);
}

pingpong_topic = new ROSLIB.Topic({
    ros : ros,
    name : '/pingpong',
    messageType : 'std_msgs/String'
});

pingpong_message = new ROSLIB.Message({
    data: "pong " + String(Date.now() / 1000)
});

var listener = new ROSLIB.Topic({
    ros : ros,
    name : '/pingpong',
    messageType : 'std_msgs/String'
});

listener.subscribe(function(message) {
    console.log(message)
    document.getElementById("pingpong_status_light").style.opacity = "1";
});

setInterval(decayOpacity, 50);