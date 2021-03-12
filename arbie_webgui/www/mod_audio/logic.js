
const constraints = window.constraints = {
    audio: true,
    video: false
  };



var prefix = "data:audio/mp3;base64, SUQzBAAAAAAAI1RTU0UAAAAPAAADTGF2ZjU4LjEzLjEwMAAAAAAAAAAAAAAA";
var count = 1;
var suffix = "VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV"

var audiostream;
var audiochunks = "";
var audioChunks = "";
var gapaudiochunks = "";
var audioplay = new Audio();
var audiogap = new Audio();
var initialised = 0;
var stop_button = 0;

var AudioContext = window.AudioContext || window.webkitAudioContext;
var audioContext;
var audioRecorder;
var source;
var base64data;

//For the visualizer
var paths = document.getElementsByTagName('path');
var visualizer = document.getElementById('visualizer');
var mask = visualizer.getElementById('mask');
var h = document.getElementsByTagName('h1')[0];
var path;
var audioSrc;
var analyser;

function audio_init() {
  document.getElementById('speech_in').style.opacity = "0.0";
  document.getElementById('speech_out').style.opacity = "0.0";
  console.log("Audio streaming module loaded.")
}

function receive() {

  document.getElementById('speech_out').style.opacity = "0.0";
  audiostream = new ROSLIB.Topic({
    ros: ros,
    name: '/audio/audio',
    messageType: 'audio_common_msgs/AudioData'
  });
  
        //Visualizer
        h.setAttribute('style', 'opacity: 0;');
        var audioContext = new AudioContext();

        if(initialised == 0){
          audioSrc = audioContext.createMediaElementSource(audioplay);
          analyser = audioContext.createAnalyser();
          audioSrc.connect(analyser);
          audioSrc.connect(audioContext.destination);
        }
        analyser.fftSize = 128;
  
        var frequencyArray = new Uint8Array(analyser.frequencyBinCount);
        visualizer.setAttribute('viewBox', '0 0 128 128');
        
        //Through the frequencyArray has a length longer than 255, there seems to be no
        //significant data after this point. Not worth visualizing.
        for (var i = 0 ; i < 128; i++) {
            path = document.createElementNS('http://www.w3.org/2000/svg', 'path');
            path.setAttribute('stroke-dasharray', '4,1');
            mask.appendChild(path);
          }
        var doDraw = function () {
            requestAnimationFrame(doDraw);
            analyser.getByteFrequencyData(frequencyArray);
            var adjustedLength;
            for (var i = 0 ; i < 128; i++) {
                adjustedLength = Math.floor(frequencyArray[i]) - (Math.floor(frequencyArray[i]) % 5);
                paths[i].setAttribute('d', 'M '+ (i) +',128 l 0,-' + adjustedLength);
            }
  
        }
        doDraw();


  audiostream.subscribe(function(message) {
    if(typeof message.data == "string"){
      
      document.getElementById('speech_in').style.opacity = "0.9";

    

    //Concatenate audiochunks
    if(count==10){

      audiochunks = prefix.concat(audiochunks);
      audiochunks = audiochunks.concat(suffix);

      initialised = 1;
      audioplay.src = audiochunks;
      audioplay.play();

      audiochunks = "";
      audiochunks = audiochunks.concat(message.data);
      gapaudiochunks = gapaudiochunks.concat(message.data);
    }
    
    else{
      audiochunks = audiochunks.concat(message.data);
      
      if(count == 2 && initialised == 1)
      {
        gapaudiochunks = prefix.concat(gapaudiochunks);
        gapaudiochunks = gapaudiochunks.concat(suffix);

        audiogap.src = gapaudiochunks;
        audiogap.play();

        gapaudiochunks = "";
        gapaudiochunks = gapaudiochunks.concat(message.data);
      }

      else
        gapaudiochunks = gapaudiochunks.concat(message.data);
    }
    

    if (count<10)
    count = count + 1;
    else
    count = 1;

    if(stop_button == 1){
      stop_button = 0;
      audiostream.unsubscribe();
      audiostream = null;
      count = 1;
      audiochunks = "";
      gapaudiochunks ="";
      document.getElementById('speech_in').style.opacity = "0.0";
      return;
    }
  }
  });

}
  
  
function broadcast(){
  document.getElementById('speech_in').style.opacity = "0.0";
  audioContext = new AudioContext();
  
  audiostream = new ROSLIB.Topic({
    ros: ros,
    name: '/audio/audio',
    messageType: 'audio_common_msgs/AudioData'
  });

  let options = { audio: true, video: false };
  navigator.mediaDevices.getUserMedia(options).then(stream => {

    //Visualizer
    h.setAttribute('style', 'opacity: 0;');
    var audioContent = new AudioContext();
    var audioStream = audioContent.createMediaStreamSource( stream );
    var analyser = audioContent.createAnalyser();
    audioStream.connect(analyser);
    analyser.fftSize = 128;

    var frequencyArray = new Uint8Array(analyser.frequencyBinCount);
    visualizer.setAttribute('viewBox', '0 0 128 128');
      
		//Through the frequencyArray has a length longer than 255, there seems to be no
    //significant data after this point. Not worth visualizing.
    for (var i = 0 ; i < 128; i++) {
        path = document.createElementNS('http://www.w3.org/2000/svg', 'path');
        path.setAttribute('stroke-dasharray', '4,1');
        mask.appendChild(path);
      }
    var doDraw = function () {
        requestAnimationFrame(doDraw);
        analyser.getByteFrequencyData(frequencyArray);
        var adjustedLength;
        for (var i = 0 ; i < 128; i++) {
            adjustedLength = Math.floor(frequencyArray[i]) - (Math.floor(frequencyArray[i]) % 5);
            paths[i].setAttribute('d', 'M '+ (i) +',128 l 0,-' + adjustedLength);
        }

    }
    doDraw();

    //transmitting the signal
    source = audioContext.createMediaStreamSource(stream);
      audioRecorder = new WebAudioRecorder(source, {
        workerDir: 'mod_audio/js/',
          encoding: 'mp3',
          } 
        );
      audioRecorder.setOptions({
        timeLimit: 0.5,
        encodeAfterRecord: false,
        mp3: { bitRate: 64 }
      })


      audioRecorder.onComplete = (audioRecorder, blob) => {

        document.getElementById('speech_out').style.opacity = "0.9";

        var reader = new FileReader();
        reader.readAsDataURL(blob); 
        reader.onloadend = function() {
        base64data = reader.result;                
          }
        if(audiostream != null){

          var audioMessage = new ROSLIB.Message({
            data : base64data
          })
          
          audiostream.publish(audioMessage);
          console.log("sent");
        }

        if(stop_button == 1){
          stop_button = 0;
          audioRecorder.finishRecording();
          stream.getAudioTracks()[0].stop();

          document.getElementById('speech_out').style.opacity = "0.0";

          return;
        }
        audioRecorder.startRecording();
      }
      audioRecorder.startRecording();

});
}

function stop(){

      stop_button = 1;
}
