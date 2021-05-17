import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:rive/rive.dart';

class ScrollAnimation extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return FutureBuilder<Artboard?>(
        future: _getArtboard(),
        //future: Future.delayed(Duration(seconds: 1)),
        builder: (context, artboard) {
          if (artboard.connectionState == ConnectionState.done) {
            return Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                RotatedBox(
                  quarterTurns: 1,
                  child: Text(
                    'Scroll',
                    style: TextStyle(color: Colors.white, fontSize: 14),
                  ),
                ),
                SizedBox(
                  height: 10,
                ),
                Container(
                    height: 50,
                    width: 50,
                    //color: Colors.blue,
                    child: Rive(
                      artboard: artboard.data!,
                    )),
              ],
            );
          } else
            return SizedBox.shrink();
        });
  }

  Future<Artboard>? _getArtboard() {
    return rootBundle.load('assets/animations/005_scroll.riv').then((data) {
      return RiveFile.import(data).mainArtboard
        ..addController(SimpleAnimation('idle')..isActive = true);
    }).catchError((e) {
      print(e);
    });
  }
}
