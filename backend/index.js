const express = require('express')
const router = require("./router")
const app = express()
const port = 3000

app.use((req, res, next) => {
  //접속 기록을 남겨주는 미들웨어
  console.log("Path : ", req.url)
  next()
})

app.use("/", router)

app.listen(port, () => {
  console.log(`Example app listening at http://localhost:${port}`)
})