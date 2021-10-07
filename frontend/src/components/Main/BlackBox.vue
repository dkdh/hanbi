<template>
    <el-card id="LostCont" shadow="always" :body-style="{ padding: '20px' }">
        <el-row>
            <el-col :span="6" v-for="(video, idx) in test" :key="idx" style="margin-right:110px">
                <video  width="320" height="240" :src="video.fileUrl" class="image" controls>
                </video>
                <div style="padding: 15px">
                  <span>{{ video.createdAt }}</span>
                </div>
            </el-col>
        </el-row>
    </el-card>
</template>

<script>
import "@/assets/css/LostCont.css";
import axios from "axios"

export default {
  data: function() {
    return {
      videos : [],
      test : [
              {fileUrl: "https://iot-hanbi.s3.ap-northeast-2.amazonaws.com/8501760.webm", createdAt: "aaaaaaaaaaaaaaaaa"},
              {fileUrl: "https://iot-hanbi.s3.ap-northeast-2.amazonaws.com/8501760.webm", createdAt: "aaaaaaaaaaaaaaaaa"},
              {fileUrl: "https://iot-hanbi.s3.ap-northeast-2.amazonaws.com/8501760.webm", createdAt: "aaaaaaaaaaaaaaaaa"},
              {fileUrl: "https://iot-hanbi.s3.ap-northeast-2.amazonaws.com/8501760.webm", createdAt: "aaaaaaaaaaaaaaaaa"},
              {fileUrl: "https://iot-hanbi.s3.ap-northeast-2.amazonaws.com/8501760.webm", createdAt: "aaaaaaaaaaaaaaaaa"},
              ]
    }
  },
  methods: {
    getBlackBox() {
      axios.get("http://j5a102.p.ssafy.io/api/record")
      .then(res => {
        this.videos = res.data
      })
      .catch(err => {
        console.log(err)
      })
    },
  },
  created() {
    this.getBlackBox()
  },


};
</script>
<style>
</style>