import 'dart:ui';

import 'package:dorvan/presentation/hud.dart';
import 'package:dorvan/presentation/main_scroll_view.dart';
import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

class MyApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return ProviderScope(
        child: MaterialApp(
      debugShowCheckedModeBanner: false,
      //theme: ThemeData(fontFamily: 'Sf'),
      home: Scaffold(
        body: Stack(
          children: [
            MainScrollView(),
            Hud(),
          ],
        ),
      ),
    ));
  }
}
