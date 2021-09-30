const express = require('express')
const router = require("./router")
const app = express()
const port = 3000
const server = require('http').createServer(app);
const io = require("./utils/socketio").createSocket(server)

require("./utils/init")

app.use((req, res, next) => {
  //접속 기록을 남겨주는 미들웨어
  console.log("Path : ", req.url)
  next()
})

app.use("/api", router)

server.listen(port, () => {
  console.log(`Example app listening at http://localhost:${port}`)
})


// const fs = require("fs")
// path = "H:\\Project\\Ssafy_2\\PJT2\\sbj3\\S05P21A102\\backend\\assets\\"
// var inF = fs.createReadStream(path + 'aaa.txt', { flags: 'a+' });
// var outF = fs.createWriteStream(path + 'bbb.txt', { flags: 'w' });

// inF.on('data', function (fileData) {
//   console.log("읽기 :" + fileData); //""+ 문자열인지 인지를 하지 않으면 출력 안나옴
//   outF.write(fileData);
// });

// inF.on('end', function () {
//   console.log("읽기 종료");
//   outF.end(function () {
//     console.log("쓰기 종료")
//   });
// })


// // for (i = 0; i < 10; i++) {
// //   fs.writeFileSync(fd, String(i))
// //   fs.of
// // }