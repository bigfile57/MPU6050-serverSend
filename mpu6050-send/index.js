//설치 모듈 : body-parser 
let express = require("express"),
  http = require("http"),
  path = require("path");
let app = express();
let bodyParser = require("body-parser");
//const fs = require('fs');
app.use(bodyParser.json());
app.use(bodyParser.urlencoded({
  extended: true,
}));

// app.engine("html", require("ejs").renderFile);
// app.set("views", path.join(__dirname, "views"));
// app.set("view engine", "ejs");

//Express 서버 시작
app.listen(3000, function () {
  console.log("start! express server is running on port 3000");
});

//클라이언트에서 주소:3000/ 접속 시 페이지 렌더
app.get("/", function (req, res) {
  res.render(__dirname + "/zyroSensor.html");
});

//POST 요청을 받으면 req 값 추출해서 각 변수에 저장 후 로그출력
app.post("/senser_data", function (req, res) {
  let Yaw = req.body.value.Yaw;
  let Pitch = req.body.value.Pitch;
  let Roll = req.body.value.Roll;
  console.log(Yaw, Pitch, Roll);
  res.sendStatus(200); // 요청성공 응답 보내주지 않으면 ESP32 오류 (데이터가 잘 안옴)
});



// app.post("/ajax_send", function (req, res) {

// });
// const saveJson = function (yaw, pitch, roll) {
//   jData.Yaw = yaw
//   jData.Pitch = pitch
//   jData.Roll = roll

//   let jsonData = JSON.stringify(jData)
//   fs.writeFileSync('sensor-json.json', jsonData)

// }