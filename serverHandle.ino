


//format bytes
String formatBytes(size_t bytes) {
  if (bytes < 1024) {
    return String(bytes) + "B";
  } else if (bytes < (1024 * 1024)) {
    return String(bytes / 1024.0) + "KB";
  } else if (bytes < (1024 * 1024 * 1024)) {
    return String(bytes / 1024.0 / 1024.0) + "MB";
  } else {
    return String(bytes / 1024.0 / 1024.0 / 1024.0) + "GB";
  }
}

String getContentType(String filename) {
  if (server.hasArg("download")) return "application/octet-stream";
  else if (filename.endsWith(".bin")) return "application/octet-stream";
  else if (filename.endsWith(".htm")) return "text/html";
  else if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".png")) return "image/png";
  else if (filename.endsWith(".gif")) return "image/gif";
  else if (filename.endsWith(".jpg")) return "image/jpeg";
  else if (filename.endsWith(".ico")) return "image/x-icon";
  else if (filename.endsWith(".xml")) return "text/xml";
  else if (filename.endsWith(".pdf")) return "application/x-pdf";
  else if (filename.endsWith(".zip")) return "application/x-zip";
  else if (filename.endsWith(".gz")) return "application/x-gzip";
  return  F( "text/plain");
}

bool handleFileRead(String path) {
  DBG_OUTPUT_PORT.println("handleFileRead: " + path);
  if (path.endsWith("/")) path += "index.htm";
  String contentType = getContentType(path);
  String pathWithGz = path + ".gz";
  if (SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)) {
    if (SPIFFS.exists(pathWithGz))
      path += ".gz";
    File file = SPIFFS.open(path, "r");
    size_t sent = server.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}

void handleFileUpload() {
  if (server.uri() != "/edit") return;
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    String filename = upload.filename;
    if (!filename.startsWith("/")) filename = "/" + filename;
    DBG_OUTPUT_PORT.print("handleFileUpload Name: "); DBG_OUTPUT_PORT.println(filename);
    fsUploadFile = SPIFFS.open(filename, "w");
    filename = String();
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    //DBG_OUTPUT_PORT.print("handleFileUpload Data: "); DBG_OUTPUT_PORT.println(upload.currentSize);
    if (fsUploadFile)
      fsUploadFile.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (fsUploadFile)
      fsUploadFile.close();
    DBG_OUTPUT_PORT.print("handleFileUpload Size: "); DBG_OUTPUT_PORT.println(upload.totalSize);
  }
}

void handleFileDelete() {
  if (server.args() == 0) return server.send(500,  F( "text/plain"),  F( "BAD ARGS"));
  String path = server.arg(0);
  DBG_OUTPUT_PORT.println("handleFileDelete: " + path);
  if (path == "/")
    return server.send(500,  F( "text/plain"), "BAD PATH");
  if (!SPIFFS.exists(path))
    return server.send(404,  F( "text/plain"), "FileNotFound");
  SPIFFS.remove(path);
  server.send(200,  F( "text/plain"), "");
  path = String();
}

void handleFileCreate() {
  if (server.args() == 0)
    return server.send(500,  F( "text/plain"),  F( "BAD ARGS"));
  String path = server.arg(0);
  DBG_OUTPUT_PORT.println("handleFileCreate: " + path);
  if (path == "/")
    return server.send(500,  F( "text/plain"), "BAD PATH");
  if (SPIFFS.exists(path))
    return server.send(500,  F( "text/plain"), "FILE EXISTS");
  File file = SPIFFS.open(path, "w");
  if (file)
  {
    //file.println("test");
    //file.flush();
    file.close();
  }
  else
    return server.send(500,  F( "text/plain"), "CREATE FAILED");
  server.send(200,  F( "text/plain"), "");
  path = String();
}


void handleFileList() {
  if (!server.hasArg("dir")) {
    server.send(500,  F( "text/plain"),  F( "BAD ARGS"));
    return;
  }

  String path = server.arg("dir");
  DBG_OUTPUT_PORT.println("handleFileList: " + path);
  Dir dir = SPIFFS.openDir(path);
  path = String();

  String output = "[";
  while (dir.next()) {
    File entry = dir.openFile("r");
    if (output != "[") output += ',';
    bool isDir = false;
    output += "{\"type\":\"";
    output += (isDir) ? "dir" : "file";
    output += "\",\"name\":\"";
    output += String(entry.name()).substring(1);
    output += "\"}";
    entry.close();
  }

  output += "]";
  server.send(200, "text/json", output);
}

void httpUpdate() {

  if (server.hasArg("version")) {
    // Use WiFiClient class to create TCP connections
    WiFiClient client;
    const int httpPort = 80;
    if (!client.connect(updatehost, httpPort)) {
      Serial.println("connection failed");
      return;
    }



    // We now create a URI for the request
    String url = "/esp8266/update/DataLogger.ver";
    Serial.print("Requesting URL: ");
    Serial.println(url);

    // This will send the request to the server
    client.print(String("GET ") + url + " HTTP/1.1\r\n" +
                 "Host: " + host + "\r\n" +
                 "Connection: close\r\n\r\n");
    unsigned long timeout = millis();
    while (client.available() == 0) {
      if (millis() - timeout > 5000) {
        Serial.println(">>> Client Timeout !");
        client.stop();
        return;
      }
    }

    // Read all the lines of the reply from server and print them to Serial
    String line;
    while (client.available()) {
      line += client.readStringUntil('\r');
    }
    int firstJson = line.indexOf("{");

    line.replace("}", ",\"syscore\":\"" + String( F(__DATE__))  + " " +  String( F(__TIME__) ) + "\"}");

    line = line.substring(firstJson);

    client.stop();
    server.send(200,  F( "text/json"), line);

    return;
  }

  if (server.hasArg("core")) {

    // blinking LED
    for (uint8_t i = 0 ; i < 200 ; i++) {
      digitalWrite(LED1,  i & 0x1 );
      rtc.writeSqwPinMode( modes[i & 0x1]);
      delay(50);
    }
    rtc.writeSqwPinMode(SquareWave1HZ);

    String url = "http://" ;
    url += updatehost;
    url += "/esp8266/update/DataLogger.ino.generic.bin";

    t_httpUpdate_return ret = ESPhttpUpdate.update(url);
    switch (ret) {
      case HTTP_UPDATE_FAILED:
        server.send(500,  F( "text/plain"),  F( "Update failed"));
        break;

      case HTTP_UPDATE_NO_UPDATES:
        server.send(200,  F( "text/plain"), "HTTP_UPDATE_NO_UPDATES");
        break;

      case HTTP_UPDATE_OK:
        server.send(200,  F( "text/plain"), "HTTP_UPDATE_OK");
        break;
    }
  }

  if (server.hasArg("spiffs")) {

    
    for (uint8_t i = 0 ; i < 200 ; i++) {
      digitalWrite(LED1,  i & 0x1 );
      rtc.writeSqwPinMode( modes[i & 0x1]);
      delay(50);
    }
    rtc.writeSqwPinMode(SquareWave1HZ);
    
    String url = "http://" ;
    url += updatehost;
    url += "/esp8266/update/DataLogger.spiffs.bin";

    t_httpUpdate_return ret = ESPhttpUpdate.updateSpiffs(url);
    if (ret == HTTP_UPDATE_OK) {
      USE_SERIAL.println("Update sketch...");
      ret = ESPhttpUpdate.update(url);

      switch (ret) {
        case HTTP_UPDATE_FAILED:
          server.send(500,  F( "text/plain"),"HTTP_UPDATE_FAILED" );
          break;

        case HTTP_UPDATE_NO_UPDATES:
          server.send(500,  F( "text/plain"), "HTTP_UPDATE_NO_UPDATES");
          break;

        case HTTP_UPDATE_OK:
          server.send(200,  F( "text/plain"), "HTTP_UPDATE_OK");
          break;
      }
      return;
    }
 server.send(500,  F( "text/plain"), "HTTP_UPDATE_FAILED");
  }

  return;

}



void adjustRTC() {
  if (!server.hasArg("unixtime")) {
    server.send(500,  F( "text/plain"),  F( "BAD ARGS"));
    return;
  }
  uint32_t utime = server.arg("unixtime").toInt();
  if (utime < 1463833206) {
    server.send(500,  F( "text/plain"),  F( "BAD ARGS"));
    return;
  }

  rtc.adjust(DateTime(utime));
  server.send(200,  F( "text/plain"), "OK");

  //startTime = rtc.now().unixtime() - millis() / 1000;
  startTime =  utime - millis() / 1000;
  // delay(500);
  // ESP.restart(); // restarts system
}

void getConfig() {
  String json = "{";

  if (sysConfig.logDataON)
    json += "\"recording\":\"on\"" ;
  else
    json += "\"recording\":\"off\"" ;

  if (sysConfig.diskFull)
    json += ", \"diskFull\":\"true\"" ;
  else
    json += ", \"diskFull\":\"false\"";

  json += ", \"interval\":" +  String(sysConfig.samplingInterval);
  json += "}";
  server.send(200, "text / json", json);
  // server.send(200, "text/plain", json);
  json = String();
}

void setConfig() {
  if (server.hasArg("interval")) {
    uint32_t value = server.arg("interval").toInt();
    if (value >= 100) {
      sysConfig.samplingInterval = value;
      saveSysConfig();
      server.send(200,  F( "text/plain"), String(sysConfig.samplingInterval));
    }
  }

  if (server.hasArg("record")) {
    sysConfig.logDataON = server.arg("record") == "on";
    saveSysConfig();
    server.send(200,  F( "text/plain"), "OK");
    checkSystemStatus();
  }

}

void sysinfo() {

  FSInfo df;
  SPIFFS.info(df);

  String json = "{";
  json += "\"interval\":" +  String(sysConfig.samplingInterval);

  json += "}";
  server.send(200, "text / json", json);
  json = String();
}

