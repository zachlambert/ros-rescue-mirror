function rand_id() {
    return Math.random().toString(36).replace('0.', '')
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

loadModule("gamepad");
loadModule("odrive");
loadModule("power");