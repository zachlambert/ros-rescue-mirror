function rand_id() {
    return Math.random().toString(36).replace('0.', '')
}

var elem = document.documentElement;
var screenStatus = 0;
function fullScreen(){
    if (elem.requestFullscreen && screenStatus == 0) {
        elem.requestFullscreen();
        screenStatus =1;
    }
    else if(document.exitFullscreen && screenStatus == 1){
        document.exitFullscreen();
        screenStatus =0;
    }
}

function loadModule(name) {
    let id = rand_id();
    addCSS("mod_" + name + "/style.css");
    addHTML("mod_" + name + "/content.html", name, id)
}

function addCSS(path) {
    var link = document.createElement('link');
    link.setAttribute('rel', 'stylesheet');
    link.setAttribute('type', 'text/css');
    link.setAttribute('href', path);
    document.getElementsByTagName('head')[0].appendChild(link);
}

function addJS(path, name, id) {
    let script = document.createElement('script');
    script.setAttribute("type", "text/javascript");
    script.setAttribute("src", path);
    script.onload = function() { eval(name + "_init('" + id + "')")};
    document.getElementsByTagName("head")[0].appendChild(script);
}

function addHTML(path, name, id) {
    let xhttp = new XMLHttpRequest();

    xhttp.onload = function (e) {
        let div = document.createElement("div");
        div.className = "mod_" + name;
        div.setAttribute("id", "mod_" + name + "_" + id);
        div.innerHTML = xhttp.response;
        document.getElementById("modules").appendChild(div);
        addJS("mod_" + name + "/logic.js", name, id)
    };

    xhttp.open("GET", path);
    xhttp.send();
}

window.addEventListener("keypress", function (e) {
    if(e.key === "#") {
        if(document.getElementById("settings").style.opacity === "0") {
            document.getElementById("settings").style.opacity = "1";
            document.getElementById("settings").style.pointerEvents = "initial"
        } else {
            document.getElementById("settings").style.opacity = "0";
            document.getElementById("settings").style.pointerEvents = "none"
        }
    }
});

function camera_init(id) {
    if(typeof(videoElements) === "undefined") {
        videoElements = {}
    }

    if(typeof(videoSelects) === "undefined") {
        videoSelects = {}
    }

    if(typeof(videoStreams) === "undefined") {
        videoStreams = {}
    }


    videoElements[id] = document.getElementById("mod_camera_" + id).getElementsByClassName("camera_stream")[0];
    if (navigator.mediaDevices.getUserMedia) {
        navigator.mediaDevices.getUserMedia({video: true})
            .then(function (stream) {
                videoElements[id].srcObject = stream;
            })
            .catch(function (err0r) {
                console.log("Something went wrong!", err0r);
            });
    }

    navigator.mediaDevices.enumerateDevices().then(function(e) { gotDevices(e, id) }).then(function(e) { getStream(e, id) }).catch(handleError);

    videoSelects[id] = document.getElementById("mod_camera_" + id).getElementsByClassName("video_selector")[0];
    videoSelects[id].onchange = function(e) { getStream(e, id) };
}

function gotDevices(deviceInfos, id) {
    for (let i = 0; i !== deviceInfos.length; ++i) {
        const deviceInfo = deviceInfos[i];
        const option = document.createElement('option');
        option.value = deviceInfo.deviceId;
        if (deviceInfo.kind === 'videoinput') {
            option.text = deviceInfo.label || 'camera ' + (videoSelects[id].length + 1);
            videoSelects[id].appendChild(option);
        }
    }
}

function getStream(e, id) {
    if (videoStreams[id]) {
        videoStreams[id].getTracks().forEach(function (track) {
            track.stop();
        });
    }

    console.log(id)

    const constraints = {
        video: {
            deviceId: {exact: videoSelects[id].value}
        }
    };

    navigator.mediaDevices.getUserMedia(constraints).then(function(e) { gotStream(e, id) }).catch(handleError);
}

function gotStream(stream, id) {
    window.stream = stream; // make stream available to console
    videoElements[id].srcObject = stream;
}

function handleError(error) {
    console.error('Error: ', error);
}

ros = new ROSLIB.Ros({
    url: 'ws://' + window.location.hostname + ':9090'
});

ros.on('connection', function () {
    console.log('Connected to websocket servier.');
});

ros.on('error', function (error) {
    console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function () {
    console.log('Connection to websocket server closed.');
});

/*
var elem = document.documentElement;
var fullscreenState = 0;



function toggleFullscreen() {
    if(fullscreenState == 0){
        if (elem.requestFullscreen) {
            elem.requestFullscreen();
            fullscreenState = 1;
      }
    }
    else if(fullscreenState == 1){
      if (document.exitFullscreen) {
          document.exitFullscreen();
        fullscreenState = 0;
      }
    }
}
*/