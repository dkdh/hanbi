<template>
  <el-card
    id="LogPhotoCont"
    shadow="always"
    :body-style="{ padding: '20px' }"
    style="overflow-y: auto"
  >
    <div id="logPhotoWrap" style="width: 100%">
      <div>
        <el-card
          :body-style="{ padding: '0px' }"
          v-for="e in elements"
          :key="e"
        >
          <img :src="e.fileUrl" class="image" />
          <div style="padding: 14px">
            <span>{{ e.createdAt }}</span>
            <div class="bottom clearfix">
              <time class="time">{{ e.caseString }}</time>
              <!-- <el-button type="text" class="button">Operating</el-button> -->
            </div>
          </div>
        </el-card>
      </div>
    </div>
  </el-card>
</template>

<script>
import "@/assets/css_kjh/LogPhotoCont.css";
import axios from "axios";

export default {
  data: function () {
    return {
      elements: [],
    };
  },
  methods: {
    getPicture() {
      axios
        .get("http://j5a102.p.ssafy.io/api/picture")
        .then((res) => {
          this.elements = res.data;
        })
        .catch((err) => {
          console.log(err);
        });
    },
  },
  created() {
    this.getPicture()
  },
};
</script>
<style>
</style>