import 'package:dorvan/application/providers/page_controller_provider.dart';
import 'package:dorvan/domain/entity/article.dart';
import 'package:dorvan/domain/entity/intro.dart';
import 'package:dorvan/presentation/white_button.dart';
import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

import 'aware_text.dart';

class IntroView extends StatelessWidget {
  final Intro? intro;

  IntroView({@required this.intro});

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.all(30.0),
      child: Stack(
        children: [
          Align(
            alignment: Alignment(-1, -0.5),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                AwareText(
                  intro!.title,
                  builder: (text) => Text(
                    text,
                    style: TextStyle(color: Colors.white, fontSize: 30),
                  ),
                ),
                SizedBox(height: 20),
                AwareText(
                  intro!.description,
                  builder: (text) => Text(
                    text,
                    style: TextStyle(color: Colors.white70, fontSize: 14),
                  ),
                ),
              ],
            ),
          ),
          Align(
            alignment: Alignment(0.0, 0.5),
            child: WhiteButton(
                onTap: () {
                  context.read(pageControllerProvider).animateToPage(2,
                      duration: Duration(milliseconds: 300),
                      curve: Curves.easeInOutCubic);
                },
                text: 'start'),
          )
        ],
      ),
    );
  }
}
