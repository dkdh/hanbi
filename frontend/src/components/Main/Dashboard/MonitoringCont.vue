<template>
  <el-card style="background-color: white; height: 100%">
    <div slot="header">
      <span>실시간 화면</span>
    </div>
    <canvas id="streamingCanvas" width="320" height="240"></canvas>
    <br />
    <el-button type="danger" @click="clickRecordBtn">{{ btnText }}</el-button>
  </el-card>
</template>

<script>
import axios from "axios";
import { mapState } from "vuex";

const API_ENDPOINT = "https://77lrzjpuy7.execute-api.ap-northeast-2.amazonaws.com/default/getVideoUrl"

let canvas;
let ctx;
let videoStream;
let mediaRecorder;
let chunks;
let url;

export default {
  data: function () {
    return {
      btnText: "녹화 시작",
    };
  },
  methods: {
    clickRecordBtn: function () {
      if (this.btnText == "녹화 시작") {
        mediaRecorder.start();
        this.btnText = "녹화 중지";
      } else {
        mediaRecorder.stop();
        // 1초 후 요청
        setTimeout(() => {
          this.$store.dispatch("stopRecord", url);
        }, 1000);
        this.btnText = "녹화 시작";
      }
    },
    changeScreen: function () {
      let bytes = new Uint8Array(this.streaming);
      let blob = new Blob([bytes], { type: "image/jpeg" });
      let urlCreator = window.URL || window.webkitURL;
      let imageUrl = urlCreator.createObjectURL(blob);

      var newImg = new Image();
      newImg.src = imageUrl;
      newImg.onload = function () {
        ctx.drawImage(newImg, 0, 0, 320, 240);
      };
    },
  },
  computed: {
    ...mapState(["streaming"]),
  },
  watch: {
    streaming: "changeScreen",
  },
  mounted() {
    canvas = document.querySelector("#streamingCanvas");
    ctx = canvas.getContext("2d");
    videoStream = canvas.captureStream(150);
    mediaRecorder = new MediaRecorder(videoStream);
    chunks = [];
    mediaRecorder.ondataavailable = function (e) {
      chunks.push(e.data);
    };
    mediaRecorder.onstop = async function (e) {
      console.log(e);
      let blob = new Blob(chunks, { type: "video/webm" });
      chunks = [];

      const response = await axios({
        method: "GET",
        url: API_ENDPOINT,
      });
      console.log("Response: ", response);
      // Put request for upload to S3
      const result = await fetch(response.data.uploadURL, {
        method: "PUT",
        body: blob,
      });
      console.log("Result: ", result);

      let fileKey = response.data.Key;
      url = "https://iotiothanbi.s3.ap-northeast-2.amazonaws.com/" + fileKey;
      console.log(url);
    };
  },
};
</script>

<style>
</style>