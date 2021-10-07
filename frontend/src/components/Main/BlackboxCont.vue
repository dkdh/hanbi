<template>
  <el-card id="BlackboxCont" shadow="always" :body-style="{ padding: '20px' }">
    <div id="blackboxWrap" style="width: 100%">
      <div>
        <video
          v-for="(video, idx) in videos"
          :key="idx"
          width="320"
          height="240"
          :src="video.fileUrl"
          class="image"
          controls
        ></video>
        <div style="padding: 15px">
          <span>{{ video.createdAt }}</span>
        </div>
      </div>
    </div>
  </el-card>
</template>

<script>
import "@/assets/css_kjh/BlackboxCont.css";
import axios from "axios";

export default {
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