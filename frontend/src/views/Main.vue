<template>
  <div id="Main" class="bc" style="height: 100%">
    <Nav></Nav>
    <div id="mainView">
      <side-bar></side-bar>
      <Main></Main>
    </div>
  </div>
</template>
<script>
import Nav from "../components/Nav.vue";
import Main from "../components/Main/MainCont.vue";
import SideBar from "../components/Sidebar.vue";
import { mapActions } from "vuex";
import "../assets/css/Main.css";
export default {
  computed: {},
  components: {
    SideBar,
    Nav,
    Main,
  },
  methods: {
    ...mapActions(["setSockets"]),
  },
  mounted() {
    this.setSockets();
  },
};
</script>