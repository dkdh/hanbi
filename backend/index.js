const express = require('express')
const router = require("./router")
const app = express()
const port = 3000
const server = require('http').createServer(app);
const io = require("./utils/socketio").createSocket(server)
const cors = require("cors")
const bodyParser = require("body-parser")

app.use(cors({

  //cors 이슈 핸들링 : 서버에 접근할 도메인을 명시
  origin: "http://localhost:8080",
}))

app.use(bodyParser.json())
app.use((req, res, next) => {
  //접속 기록을 남겨주는 미들웨어
  console.log("Path : ", req.url)
  next()
})

app.use("/api", router)

server.listen(port, () => {
  console.log(`Example app listening at ${port}`)
})


// // for (i = 0; i < 10; i++) {
// //   fs.writeFileSync(fd, String(i))
// //   fs.of
// // }