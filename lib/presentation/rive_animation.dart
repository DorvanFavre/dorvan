import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:rive/rive.dart';

class RiveAnimation extends StatelessWidget {
  final String animation;

  RiveAnimation({this.animation = ''});

  @override
  Widget build(BuildContext context) {
    return FutureBuilder<Artboard?>(
        future: _getArtboard(),
        builder: (context, artboard) {
          if (artboard.connectionState == ConnectionState.done) {
            return Rive(
              artboard: artboard.data!,
            );
          } else
            return SizedBox.shrink();
        });
  }

  Future<Artboard>? _getArtboard() {
    return rootBundle.load('assets/animations/$animation').then((data) {
      return RiveFile.import(data).mainArtboard
        ..addController(SimpleAnimation('idle')..isActive = true);
    }).catchError((e) {
      print(e);
    });
  }
}
