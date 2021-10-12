<template>
  <el-card
    id="BlackboxCont"
    shadow="always"
    :body-style="{ padding: '20px' }"
  >
    <div id="BlackboxWrap" style="width: 100%">
      <div class="card-div">
        <el-card
          :body-style="{ padding: '0px' }"
          v-for="(video, idx) in videos"
          :key="idx"
          class="el-card__class"
        >
          <video
            style="border: 1px solid black"
            width="320"
            height="240"
            :src="video.fileUrl"
            class="eachVideo"
            controls
          ></video>
          <div style="padding: 15px">
            <span>{{ video.createdAt }}</span>
          </div>
        </el-card>
      </div>
    </div>
  </el-card>
</template>

<script>
import "@/assets/css_kjh/BlackboxCont.css";
import axios from "axios";

export default {
  // https://iotiothanbi.s3.ap-northeast-2.amazonaws.com/5790214.webm
  data: function () {
    return {
      videos: [],
    };
  },
  methods: {
    getBlackBox() {
      axios
        .get("http://j5a102.p.ssafy.io/api/record")
        .then((res) => {
          this.videos = res.data;
        })
        .catch((err) => {
          console.log(err);
        });
    },
  },
  created() {
    this.getBlackBox();
  },
};
</script>
<style>
</style>