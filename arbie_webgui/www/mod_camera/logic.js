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