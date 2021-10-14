<template>
  <div id="Main" class="bc" style="height: 100%">
    <Nav></Nav>
    <div id="mainView">
      <SideBar></SideBar>
      <MainCont></MainCont>
    </div>
  </div>
</template>
<script>
import Nav from "../components/Nav.vue";
import MainCont from "../components/Main/MainCont.vue";
import SideBar from "../components/Sidebar.vue";
import { mapActions } from "vuex";
import "../assets/css/Main.css";
export default {
  computed: {},
  components: {
    SideBar,
    Nav,
    MainCont,
  },
  methods: {
    ...mapActions(["setSockets"]),
  },
  mounted() {
    this.setSockets();
  },
};
</script>