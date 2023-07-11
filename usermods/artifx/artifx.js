
var isCEEditor = false;

function toggleCEEditor(name, segID) {
    if (isInfo) toggleInfo();
    if (isNodes) toggleNodes();
    isCEEditor = !isCEEditor;
    if (isCEEditor) populateCEEditor(name, segID);
    d.getElementById('ceEditor').style.transform = (isCEEditor) ? "translateY(0px)":"translateY(100%)";
}

function fetchAndExecute(url, name, callback, callError)
{
  fetch
  (url+name, {
    method: 'get'
  })
  .then(res => {
    if (!res.ok) {
		callError("File " + name + " not found");
    	return "";
    }
    return res.text();
  })
  .then(text => {
    callback(text);
  })
  .catch(function (error) {
    callError("Error getting " + name);
  })
  .finally(() => {
    // if (callback) setTimeout(callback,99);
  });
}

function loadLogFile(name, attempt) {
    var ceLogArea = d.getElementById("ceLogArea");
    fetchAndExecute((loc?`http://${locip}`:'.') + "/", name , function(logtext)
    {
      if (logtext == "") {
        if (attempt < 10) {
          ceLogArea.value = ("...........").substring(0, attempt + 1);
          setTimeout(() =>
          {
            loadLogFile(name, attempt + 1);
          }, 1000);
        }
        else
          ceLogArea.value = "log not found after 10 seconds";
      }
      else
        ceLogArea.value = logtext;
    }, function(error){
      showToast(error);
      console.log(error);
    });
}

function uploadFileWithText(name, text)
{
  var req = new XMLHttpRequest();
  req.addEventListener('load', function(){showToast(this.responseText,this.status >= 400)});
  req.addEventListener('error', function(e){showToast(e.stack,true);});
  req.open("POST", "/upload");
  var formData = new FormData();

  var blob = new Blob([text], {type : 'application/text'});
  var fileOfBlob = new File([blob], name);
  formData.append("upload", fileOfBlob);

  req.send(formData);
}

function saveCE(name, segID) {
    showToast("Saving " + name);

    var ceProgramArea = d.getElementById("ceProgramArea");

    uploadFileWithText("/" + name, ceProgramArea.value);

    var obj = {"seg": {"id": segID, "reset": true}};
    requestJson(obj);

    var ceLogArea = d.getElementById("ceLogArea");
    ceLogArea.value = ".";
    setTimeout(() =>
    {
        loadLogFile(name + ".log", 1);
    }, 1000);
}

function populateCEEditor(name, segID)
{
  fetchAndExecute((loc?`http://${locip}`:'.') + "/", name + ".wled", function(text)
  {
    var cn=`ARTI-FX Editor<br>
            <i>${name}.wled</i><br>
            <textarea class="ceTextarea" id="ceProgramArea">${text}</textarea><br>
            <button class="btn infobtn" onclick="toggleCEEditor()">Close</button>
            <button class="btn infobtn" onclick="saveCE('${name}.wled', ${segID})">Save and Run</button><br>
            <button class="btn infobtn" onclick="downloadCEFile('CE','${name}.wled')">Download ${name}.wled</button>
            <button class="btn infobtn" onclick="loadCETemplate('${name}')">Load template</button><br>
            <button class="btn infobtn" onclick="downloadCEFile('CE','wledv033.json')">Download wled json</button>
            <button class="btn infobtn" onclick="downloadCEFile('CE','presets.json')">Download presets.json</button><br>
            <button class="btn infobtn" onclick="location.href='https://github.com/MoonModules/WLED-Effects/tree/master/ARTIFX/wled'" type="button">ARTI-FX Library</button>
            <button class="btn infobtn btn-xs" onclick="location.href='https://mm.kno.wled.ge/moonmodules/arti-fx'" type="button">?</button><br>
            <br><i>Compile and Run Log</i><br>
            <textarea class="ceTextarea" id="ceLogArea"></textarea><br>
            <i>Run log > 3 seconds is send to Serial Ouput.</i><br>
            <a href="#" onclick="downloadCEFile('HBB','presets.json');return false;" title="Download HBaas Base presets">🥚</a>
            <a href="#" onclick="downloadCEFile('HBE','presets.json');return false;" title="Download HBaas Effects presets">🥚</a>`;

    d.getElementById('kceEditor').innerHTML = cn;

    var ceLogArea = d.getElementById("ceLogArea");
    ceLogArea.value = ".";
    loadLogFile(name + ".wled.log", 1);

  }, function(error){
    showToast(error);
    console.log(error);
  });
}

function downloadCEFile(url, name) {
    if (url == "CE") url = "https://raw.githubusercontent.com/MoonModules/WLED-Effects/master/ARTIFX/wled/";
    if (url == "HBB") url = "https://raw.githubusercontent.com/MoonModules/WLED-Effects/master/Presets/HB_PresetPack210808_32x32_16seg/Base%20pack/";
    if (url == "HBE") url = "https://raw.githubusercontent.com/MoonModules/WLED-Effects/master/Presets/HB_PresetPack210808_32x32_16seg/Effects%20pack/";

    fetchAndExecute(url, name, function(text) {
        if (name == "wledv033.json" || name == "presets.json") {
            if (!confirm('Are you sure to download/overwrite ' + name + '?'))
              return;
            uploadFileWithText("/" + name, text);
        }
        else
        {
          var ceProgramArea = d.getElementById("ceProgramArea");
          ceProgramArea.value = text;
        }
    }, function(error){
      showToast(error);
      console.log(error);
    });

    return;
  
    var request = new XMLHttpRequest();
    request.onload = function() {
      if (name == "wledv033.json" || name == "presets.json") {
          if (!confirm('Are you sure to download ' + name + '?'))
            return;
          uploadFileWithText("/" + name, request.response);
      }
      else
      {
        var ceProgramArea = d.getElementById("ceProgramArea");
        ceProgramArea.value = request.response;
      }
    }
    request.open("GET", url);
    request.send();
  }
  
function loadCETemplate(name) {
    var ceProgramArea = d.getElementById("ceProgramArea");
    ceProgramArea.value = `/*
    ARTI-FX Template
  */
  program ${name}
  {
    function renderFrame()
    {
      setPixelColor(counter, colorFromPalette(counter, counter))
    }
  }`;
  
}  