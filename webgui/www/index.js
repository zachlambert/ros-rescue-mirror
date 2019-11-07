function loadModule(name) {
    addCSS("mod_" + name + "/style.css");
    addHTML("mod_" + name + "/content.html", name);
    addJS("mod_" + name + "/logic.js");
}

function addCSS(path) {
    var link = document.createElement('link');
    link.setAttribute('rel', 'stylesheet');
    link.setAttribute('type', 'text/css');
    link.setAttribute('href', path);
    document.getElementsByTagName('head')[0].appendChild(link);
}

function addJS(path) {
    let script = document.createElement('script');
    script.setAttribute("type","text/javascript");
    script.setAttribute("src", path);
    document.getElementsByTagName("head")[0].appendChild(script);
}

function addHTML(path, name) {
    let xhttp = new XMLHttpRequest();

    xhttp.onload = function (e) {
        console.log(e);
        let div = document.createElement("div");
        div.setAttribute("id", "mod_" + name);
        div.innerHTML = xhttp.response;
        document.getElementById("modules").appendChild(div);
    };

    xhttp.open("GET", path);
    xhttp.send();
}