import 'package:dorvan/domain/entity/picture.dart';
import 'package:flutter/material.dart';

import 'aware_text.dart';

class PictureView extends StatelessWidget {
  final Picture? picture;

  PictureView({@required this.picture});

  @override
  Widget build(BuildContext context) {
    return Stack(
      fit: StackFit.expand,
      children: [
        Container(
          child: Image(
            image:
                picture?.imageAsset ?? AssetImage('assets/images/dorvan.jpg'),
            fit: BoxFit.cover,
          ),
        ),
        Container(
            decoration: BoxDecoration(
                gradient: LinearGradient(
          colors: [
            Colors.black.withOpacity(0.2),
            Colors.black.withOpacity(0),
            Colors.black.withOpacity(0.8)
          ],
          stops: [0.1, 0.5, 1],
          begin: Alignment.topCenter,
          end: Alignment.bottomCenter,
        ))),
        Align(
          alignment: Alignment.bottomLeft,
          child: Padding(
            padding: EdgeInsets.symmetric(horizontal: 30, vertical: 60),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                AwareText(
                  picture?.title ?? '',
                  builder: (text) => Text(
                    text,
                    style: TextStyle(color: Colors.white, fontSize: 20),
                  ),
                ),
                AwareText(
                  picture?.description ?? '',
                  builder: (text) => Text(
                    text,
                    style: TextStyle(color: Colors.white70, fontSize: 16),
                  ),
                )
              ],
            ),
          ),
        )
      ],
    );
  }
}
